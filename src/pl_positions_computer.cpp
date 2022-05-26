/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "pl_positions_computer.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

PowerlinePositionsComputerNode::PowerlinePositionsComputerNode(const std::string & node_name, const std::string & node_namespace)
                            : Node(node_name, node_namespace), P_v_(8) {

    this->declare_parameter<int>("positions_computation_method", 0);
    int computation_method;
    this->get_parameter("positions_computation_method", computation_method);

    this->declare_parameter<int>("buffer_size", 5);
    this->get_parameter("buffer_size", buffer_size_);

    this->declare_parameter<double>("I", 120.);
    this->get_parameter("I", I_);

    this->declare_parameter<double>("cable_distance_max", 20.);
    this->get_parameter("cable_distance_max", cable_distance_max_);

    this->declare_parameter<int>("max_LMA_iterations", 1000);
    this->get_parameter("max_LMA_iterations", max_LMA_iterations_);

    this->declare_parameter<int>("max_weighting_iterations", 1000);
    this->get_parameter("max_weighting_iterations", max_weighting_iterations_);

    this->declare_parameter<double>("lambda_first_step", 0.001);
    this->get_parameter("lambda_first_step", lambda_first_step_);

    this->declare_parameter<double>("lambda_increment_factor", 1.1);
    this->get_parameter("lambda_increment_factor", lambda_increment_factor_);

    this->declare_parameter<double>("lambda_decrement_factor", 0.9);
    this->get_parameter("lambda_decrement_factor", lambda_decrement_factor_);

    this->declare_parameter<double>("step_norm_success_threshold", 1e-8);
    this->get_parameter("step_norm_success_threshold", step_norm_success_threshold_);

    RCLCPP_INFO(this->get_logger(), "Starting %s with parameters:", node_name);
    RCLCPP_INFO(this->get_logger(), "positions_computation_method: %d", computation_method);
    RCLCPP_INFO(this->get_logger(), "I: %f", I_);
    RCLCPP_INFO(this->get_logger(), "cable_distance_max: %f", cable_distance_max_);
    RCLCPP_INFO(this->get_logger(), "max_LMA_iterations: %d", max_LMA_iterations_);
    RCLCPP_INFO(this->get_logger(), "max_weighting_iterations: %d", max_weighting_iterations_);
    RCLCPP_INFO(this->get_logger(), "lambda_increment_factor: %f", lambda_increment_factor_);
    RCLCPP_INFO(this->get_logger(), "lambda_decrement_factor: %f", lambda_decrement_factor_);
    RCLCPP_INFO(this->get_logger(), "buffer_size: %d", buffer_size_);
    RCLCPP_INFO(this->get_logger(), "step_norm_success_threshold: %d", step_norm_success_threshold_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	fetchStaticTransforms();

    mag_phasors_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasors3D>(
        "/sine_reconstructor/mag_phasors", 10,
        std::bind(&PowerlinePositionsComputerNode::magPhasorsCallback, this, std::placeholders::_1)
    );

	pl_poses_result_est_sub_ = this->create_subscription<mag_pl_detector::msg::PowerlinePosesComputationResult>(
		"/pl_positions_computer/pl_poses_est", 10, std::bind(&PowerlinePositionsComputerNode::powerlinePosesEstCallback, this, std::placeholders::_1));

    pl_dir_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pl_dir_estimator/powerline_direction_est", 10,
        std::bind(&PowerlinePositionsComputerNode::plDirCallback, this, std::placeholders::_1)
    );

    pl_poses_result_raw_pub_ = this->create_publisher<mag_pl_detector::msg::PowerlinePosesComputationResult>(
        "pl_poses_raw", 10
    );

    pl0_pose_raw_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pl0_pose_raw", 10
    );

    pl1_pose_raw_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pl1_pose_raw", 10
    );

    odometry_timer_ = this->create_wall_timer(
        10ms, std::bind(&PowerlinePositionsComputerNode::odometryCallback, this));
    
    quat_t q(1,0,0,0);
    drone_quat_ = q;
    pl_dir_quat_ = q;

    p1_(0) = 0.;
    p1_(1) = 1.5;
    p1_(2) = 1.;
    p2_(0) = 0.;
    p2_(1) = -1.5;
    p2_(2) = 2.5;

}

void PowerlinePositionsComputerNode::powerlinePosesEstCallback(mag_pl_detector::msg::PowerlinePosesComputationResult::SharedPtr msg) {

    if (p_est_mutex_.try_lock()) {

        vector_t vec0(
            msg->poses[0].position.x,
            msg->poses[0].position.y,
            msg->poses[0].position.z
        );

        vector_t vec1(
            msg->poses[1].position.x,
            msg->poses[1].position.y,
            msg->poses[1].position.z
        );

        p1_ = vec0;
        p2_ = vec1;

        p_est_mutex_.unlock();

    } 
}

void PowerlinePositionsComputerNode::plDirCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    //std::cout << "plDirCallback" << std::endl;

    quat_t q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );

    if (pl_dir_mutex_.try_lock()) {

        pl_dir_quat_ = q;

        pl_dir_mutex_.unlock();

    }
}

void PowerlinePositionsComputerNode::odometryCallback() {

    //std::cout << "odometyCallback" << std::endl;

    geometry_msgs::msg::TransformStamped tf;

	try {

		tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

	} catch(tf2::TransformException & ex) {

		RCLCPP_DEBUG(this->get_logger(), "Could not get odometry transform, frame world to drone");
		return;

	}

    quat_t quat(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z
    );

    vector_t vec(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z
    );

    if (odometry_mutex_.try_lock()) {

        drone_quat_ = quat;
        drone_vec_ = vec;

        odometry_mutex_.unlock();

    }
}

void PowerlinePositionsComputerNode::magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg) {

    projectVectors(msg->phasors);
    correctVectors(msg->phasors);

    int computation_method;
    this->get_parameter("positions_computation_method", computation_method);

    switch(computation_method) {

    default:
    case 0:
        singleCableLinearComputationMethod(msg->phasors);
        break;

    case 1:
        singleCableLinearBufferingComputationMethod(msg->phasors);
        break;

    case 2:
        twoCablesNLLSComputationMethod(msg->phasors);
        break;

    case 3:
        singleCableNLLSComputationMethod(msg->phasors);
        break;

    }

    publishPoseEstimationResult();

}

void PowerlinePositionsComputerNode::projectVectors(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors) {

    pl_dir_mutex_.lock(); {

        pl_dir_quat_hold_ = pl_dir_quat_;

    } pl_dir_mutex_.unlock();

    rotation_matrix_t pl_R = quatToMat(pl_dir_quat_hold_);
    vector_t unit_x(1,0,0);
    pl_unit_x_ = pl_R * unit_x;

    point_t origin(0,0,0);

    projection_plane_ = {
        .p = origin,
        .normal = pl_unit_x_
    };

    for (int i = 0; i < 4; i++) {

        vector_t D_Malpha_pi = (vector_t)projectPointOnPlane((point_t)v_drone_to_mags_[i], projection_plane_);
        v_proj_mags_[i] = D_Malpha_pi;

        vector_t vec(
            phasors[i].normalized_amplitudes.x,
            phasors[i].normalized_amplitudes.y,
            phasors[i].normalized_amplitudes.z
        );

        vector_t D_Malpha_vi = (vector_t)projectPointOnPlane((point_t)vec, projection_plane_);
        v_proj_norm_ampls_[i] = D_Malpha_vi;

    }

}

void PowerlinePositionsComputerNode::correctVectors(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors) {

    int misaligned_votes[] = {0,0,0,0};

    for (int i = 0; i < 4; i++) {

        vector_t D_vj(
            phasors[i].amplitudes.x,
            phasors[i].amplitudes.y,
            phasors[i].amplitudes.z
        );

        if (i == 2) {
            D_vj = -D_vj;
        }

        D_vj_vecs_[i] = D_vj;
        D_vj_norm_vecs_[i] = D_vj / D_vj.norm();

        for (int j = 0; j < i; j++) {

            // cout << to_string(D_vj_norm_vecs[i].dot(D_vj_norm_vecs[j])) << "\n";


            if (D_vj_norm_vecs_[i].dot(D_vj_norm_vecs_[j]) < 0.5) {

                misaligned_votes[i]++;
                misaligned_votes[j]++;

            }
        }
    }

    int misaligned_index = -1;

    for (int i = 0; i < 4; i++) {

        if (misaligned_votes[i] > 2) {

            misaligned_index = i;
            break;

        }
    }

    if(misaligned_index > -1) {
        int compare_index;
        double smallest_dist = 9999999.;

        for (int i = 0; i < 4; i++) {
            if (i == misaligned_index){
                continue;
            }

            vector_t dist_vec = P_D_Mja_p_[misaligned_index] - P_D_Mja_p_[i];
            double dist = dist_vec.norm();
            if (dist < smallest_dist) {
                smallest_dist = dist;
                compare_index = i;
            }
        }

        // cout << "Found misaligned index: " << to_string(misaligned_index) << "\n";
        bool finished = false;

        float angles[] = {M_PI_2, M_PI, 3*M_PI_2};

        rotation_matrix_t best_R;
        double best_dot_prod = -9999;

        for (int i = 0; i < 3; i++) {

            for (int j = 0; j < 3; j++) {

                orientation_t eul(0,0,0);
                eul(i) = angles[j];
                rotation_matrix_t R = eulToR(eul);

                vector_t vec_rotated = R * D_vj_norm_vecs_[misaligned_index];

                double dot_prod = vec_rotated.dot(D_vj_norm_vecs_[compare_index]);
                if (dot_prod > best_dot_prod) {
                    best_dot_prod = dot_prod;
                    best_R = R;
                }
            }
        }

        D_vj_vecs_[misaligned_index] = best_R * D_vj_vecs_[misaligned_index];
    }


}

void PowerlinePositionsComputerNode::singleCableLinearComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors) {

    Eigen::MatrixXf A(5, 3);
    Eigen::VectorXf b(5);

    for (int i = 0; i < 4; i++) {

        //vector_t norm_ampl(
        //    phasors[i].normalized_amplitudes.x,
        //    phasors[i].normalized_amplitudes.y,
        //    phasors[i].normalized_amplitudes.z
        //);

        //A(i, 0) = norm_ampl(0);
        //A(i, 1) = norm_ampl(1);
        //A(i, 2) = norm_ampl(2);

        A(i, 0) = v_proj_norm_ampls_[i](0);
        A(i, 1) = v_proj_norm_ampls_[i](1);
        A(i, 2) = v_proj_norm_ampls_[i](2);

        b(i) = v_proj_norm_ampls_[i].dot(v_proj_mags_[i]);

    }

    A(4, 0) = pl_unit_x_(0);
    A(4, 1) = pl_unit_x_(1);
    A(4, 2) = pl_unit_x_(2);

	Eigen::Vector3f x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    x = (Eigen::Vector3f)projectPointOnPlane((point_t)x, projection_plane_);

    geometry_msgs::msg::Pose pl_pose;

    pl_pose.orientation.w = pl_dir_quat_hold_(0);
    pl_pose.orientation.x = pl_dir_quat_hold_(1);
    pl_pose.orientation.y = pl_dir_quat_hold_(2);
    pl_pose.orientation.z = pl_dir_quat_hold_(3);

    pl_pose.position.x = x(0);
    pl_pose.position.y = x(1);
    pl_pose.position.z = x(2);

    std::vector<geometry_msgs::msg::Pose> poses;
    poses.push_back(pl_pose);
    pl_poses_ = poses;
    pl_current_ = 0;

    publish_p1_ = true;

}

void PowerlinePositionsComputerNode::singleCableLinearBufferingComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors) {

    quat_t drone_quat;
    vector_t drone_vec;

    odometry_mutex_.lock(); {

        drone_quat = drone_quat_;
        drone_vec = drone_vec_;

    } odometry_mutex_.unlock();

    rotation_matrix_t W_D_R = quatToMat(drone_quat);

    std::vector<vector_t> W_vi;
    std::vector<vector_t> W_D_Mj_p;

    for (int i = 0; i < phasors.size(); i++) {

        vector_t phasor(
            phasors[i].normalized_amplitudes.x,
            phasors[i].normalized_amplitudes.y,
            phasors[i].normalized_amplitudes.z
        );

        W_vi.push_back(W_D_R*phasor);

        W_D_Mj_p.push_back(W_D_R*v_drone_to_mags_[i]);

    }

    measurement_item_t item = {
        .W_vi = W_vi,
        .W_D_Mj_p = W_D_Mj_p,
        .W_D_p = drone_vec
    };

    measurements_buffer.Push(item);

    if (measurements_buffer.Size() < buffer_size_) {

        std::vector<geometry_msgs::msg::Pose> poses;
        pl_poses_ = poses;
        pl_current_ = 0;

        return;

    }

    rotation_matrix_t D_W_R = W_D_R.transpose();

    std::vector<vector_t> D_vi;
    std::vector<vector_t> D_Mj_p;

    for (int i = 0; i < buffer_size_; i++) {

        measurement_item_t meas_item = measurements_buffer[i];

        for (int j = 0; j < meas_item.W_vi.size(); j++) {

            vector_t D_v = D_W_R * meas_item.W_vi[j];
            vector_t D_m_p = D_W_R * (meas_item.W_D_p - drone_vec + meas_item.W_D_Mj_p[j]);

            D_vi.push_back(D_v);
            D_Mj_p.push_back(D_m_p);

        }
    }

    Eigen::MatrixXf A(D_vi.size()+1, 3);
    Eigen::VectorXf b(D_vi.size()+1);

    for (int i = 0; i < D_vi.size(); i++) {

        A(i, 0) = D_vi[i](0);
        A(i, 1) = D_vi[i](1);
        A(i, 2) = D_vi[i](2);

        b(i) = D_vi[i].dot(D_Mj_p[i]);

        //A(i, 0) = v_proj_norm_ampls_[i](0);
        //A(i, 1) = v_proj_norm_ampls_[i](1);
        //A(i, 2) = v_proj_norm_ampls_[i](2);

        //b(i) = v_proj_norm_ampls_[i].dot(v_proj_mags_[i]);

    }

    A(D_vi.size(), 0) = pl_unit_x_(0);
    A(D_vi.size(), 1) = pl_unit_x_(1);
    A(D_vi.size(), 2) = pl_unit_x_(2);

	Eigen::Vector3f x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    x = (Eigen::Vector3f)projectPointOnPlane((point_t)x, projection_plane_);

    geometry_msgs::msg::Pose pl_pose;

    pl_pose.orientation.w = pl_dir_quat_hold_(0);
    pl_pose.orientation.x = pl_dir_quat_hold_(1);
    pl_pose.orientation.y = pl_dir_quat_hold_(2);
    pl_pose.orientation.z = pl_dir_quat_hold_(3);

    pl_pose.position.x = x(0);
    pl_pose.position.y = x(1);
    pl_pose.position.z = x(2);

    std::vector<geometry_msgs::msg::Pose> poses;
    poses.push_back(pl_pose);
    pl_poses_ = poses;
    pl_current_ = 0;

    publish_p1_ = true;

}

void PowerlinePositionsComputerNode::twoCablesNLLSComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors) {

    using namespace Eigen;

    static VectorXd x(4);

    rotation_matrix_t D_P_R = quatToMat(pl_dir_quat_hold_);

    //orientation_t eul(0,0,0.5236);
    //rotation_matrix_t D_P_R = eulToR(eul);
    rotation_matrix_t D_P_R_T = D_P_R.transpose();

    vector_t P_vj[4];

    //P_v_(0) = 22.7463;
    //P_v_(1) = -14.3418;

    //P_v_(2) = 25.0550;
    //P_v_(3) = -5.4736;
    
    //P_v_(4) = 24.0326;
    //P_v_(5) = -0.4703;
    
    //P_v_(6) = 24.7111;
    //P_v_(7) = -9.3727;

    //for (int i = 0; i < 4; i++) {
    //    cout << "v_drone_to_mags[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(v_drone_to_mags_[i](j));
    //        if (j < 2) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //for (int i = 0; i < 4; i++) {
    //    cout << "v_proj_mags_[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(v_proj_mags_[i](j));
    //        if (j < 2) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //cout << "D_P_R = [\n";
    //for (int i = 0; i < 3; i++) {
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(D_P_R(i,j));
    //        if (i == 2 and j == 2) {
    //            cout << "]\n";
    //        } else {
    //            cout << ",";
    //        }
    //    }
    //    cout << "\n";
    //}

    for (int i = 0; i < 4; i++) {

        P_D_Mja_p_[i] = D_P_R_T * v_proj_mags_[i];

        vector_t D_vj = D_vj_vecs_[i];

        // cout << "D_vj(:," << to_string(i+1) << ") = [" << to_string(D_vj(0)) << " " << to_string(D_vj(1)) << " " << to_string(D_vj(2)) << "];\n";

        P_vj[i] = D_P_R * D_vj;

        P_v_(i*2) = P_vj[i](1);
        P_v_(i*2+1) = P_vj[i](2);

    }
    // cout << endl;

    vector_t p1,p2;
    p_est_mutex_.lock(); {
        p1 = p1_;
        p2 = p2_;
    } p_est_mutex_.unlock();

    vector_t P_p1_ = D_P_R_T * p1;
    vector_t P_p2_ = D_P_R_T * p2;


    //x(0) = P_p1_(1);
    //x(1) = P_p1_(2);
    //x(2) = P_p2_(1);
    //x(3) = P_p2_(2);

    x(0) = P_p1_(1);
    x(1) = P_p1_(2);
    x(2) = P_p2_(1);
    x(3) = P_p2_(2);

    //x(0) = 120.;
    //if (P_p1_.norm() < cable_distance_max_) {

    //    x(0) = P_p1_(1);
    //    x(1) = P_p1_(2);

    //} else {

    //    x(0) = 1;
    //    x(1) = 1;

    //}

    //if (P_p2_.norm() < cable_distance_max_) {

    //    x(2) = P_p2_(1);
    //    x(3) = P_p2_(2);

    //} else {

    //    x(2) = -1;
    //    x(3) = 1;

    //}

    bool success = levenbergMarquardt(x);

    if (!success) {
        I_ = -I_;
        x(0) = P_p1_(1);
        x(1) = P_p1_(2);
        x(2) = P_p2_(1);
        x(3) = P_p2_(2);

        //x(0) = -120.;
        //x(0) = P_p2_(1);
        //x(1) = P_p2_(2);
        //x(2) = P_p1_(1);
        //x(3) = P_p1_(2);
        //if (P_p1_.norm() < cable_distance_max_) {

        //    x(2) = P_p1_(1);
        //    x(3) = P_p1_(2);

        //} else {

        //    x(2) = 1;
        //    x(3) = 1;

        //}

        //if (P_p2_.norm() < cable_distance_max_) {

        //    x(0) = P_p2_(1);
        //    x(1) = P_p2_(2);

        //} else {

        //    x(0) = -1;
        //    x(1) = 1;

        //}

        success = levenbergMarquardt(x);
    }
    //bool success = false;

    //if (!success) {

    //    std::vector<geometry_msgs::msg::Pose> poses;
    //    pl_poses_ = poses;
    //    pl_current_ = 0;

    //    return;

    //}

    // I_ = x(0);
    P_p1_(0) = 0;
    P_p1_(1) = x(0);
    P_p1_(2) = x(1);

    P_p2_(0) = 0;
    P_p2_(1) = x(2);
    P_p2_(2) = x(3);

    //if (p2_.norm() < p1_.norm()) {

    //    vector_t tmp_vec = p1_;
    //    p1_ = p2_;
    //    p2_ = tmp_vec;

    //}

    std::vector<geometry_msgs::msg::Pose> poses;

    geometry_msgs::msg::Pose pose1;

    publish_p1_ = false;
    publish_p2_ = false;

    if (success) {

        vector_t p1_tmp = D_P_R * P_p1_;
        vector_t p2_tmp = D_P_R * P_p2_;

        vector_t p1_tmp_m_p1 = p1_tmp - p1;
        vector_t p1_tmp_m_p2 = p1_tmp - p2;
        vector_t p2_tmp_m_p1 = p2_tmp - p1;
        vector_t p2_tmp_m_p2 = p2_tmp - p2;

        double p1_tmp_m_p1_norm = p1_tmp_m_p1.norm();
        double p1_tmp_m_p2_norm = p1_tmp_m_p2.norm();
        double p2_tmp_m_p1_norm = p2_tmp_m_p1.norm();
        double p2_tmp_m_p2_norm = p2_tmp_m_p2.norm();

        int indices[2];

        if (p1_tmp_m_p1_norm < p1_tmp_m_p2_norm && p1_tmp_m_p1_norm < p2_tmp_m_p1_norm && p1_tmp_m_p1_norm < p2_tmp_m_p2_norm) {
            
            indices[0] = 0;
            indices[1] = 1;

        } else if (p2_tmp_m_p2_norm < p2_tmp_m_p1_norm && p2_tmp_m_p2_norm < p1_tmp_m_p2_norm) {

            indices[0] = 0;
            indices[1] = 1;

        } else {

            indices[0] = 1;
            indices[1] = 0;

        }

        if (P_p1_.norm() < cable_distance_max_) {

            if (indices[0] == 0) {

                p1 = p1_tmp;
                publish_p1_ = true;

            } else {

                p2 = p1_tmp;
                publish_p2_ = true;

            }
        }

        if (P_p2_.norm() < cable_distance_max_) {

            if (indices[0] == 0) {

                p2 = p2_tmp;
                publish_p2_ = true;

            } else {

                p1 = p2_tmp;
                publish_p1_ = true;

            }
        }

        if (publish_p1_) {

            pose1.orientation.w = pl_dir_quat_hold_(0);
            pose1.orientation.x = pl_dir_quat_hold_(1);
            pose1.orientation.y = pl_dir_quat_hold_(2);
            pose1.orientation.z = pl_dir_quat_hold_(3);
            pose1.position.x = p1(0);
            pose1.position.y = p1(1);
            pose1.position.z = p1(2);

            poses.push_back(pose1);

            p_est_mutex_.lock(); {
                p1_ = p1;
            } p_est_mutex_.unlock();

        }

        if (publish_p2_) {

            geometry_msgs::msg::Pose pose2;
            pose2.orientation = pose1.orientation;
            pose2.position.x = p2(0);
            pose2.position.y = p2(1);
            pose2.position.z = p2(2);

            poses.push_back(pose2);

            p_est_mutex_.lock(); {
                p2_ = p2;
            } p_est_mutex_.unlock();

        }
    }

    pl_poses_ = poses;
    pl_current_ = I_;

}

bool PowerlinePositionsComputerNode::levenbergMarquardt(Eigen::VectorXd &x) {

    using namespace Eigen;
    using namespace std;

    static VectorXd d(4);
    static VectorXd fx(8);
    static VectorXd fx_tmp(8);
    static MatrixXd fdx(8,4);
    static MatrixXd fdx_T(8,4);
    static MatrixXd JTJ(4,4);
    static VectorXd r(8);
    static VectorXd r_tmp(8);
    static double r_norm;
    static double r_tmp_norm;
    static VectorXd x_tmp(4);
    static double lambda = 0;
    static MatrixXd A(4,4);
    static VectorXd b(4);
    static MatrixXd eye = Eigen::MatrixXd::Identity(4,4);

    static VectorXd (PowerlinePositionsComputerNode::*f)(VectorXd x) = &PowerlinePositionsComputerNode::B;
    static MatrixXd (PowerlinePositionsComputerNode::*fd)(VectorXd x) = &PowerlinePositionsComputerNode::J;

    int cnt = 0;
    bool success = false;

    lambda = 0;
    x_tmp = x;
    fx = (this->*f)(x);
    r = P_v_ - fx;
    r_tmp = r;
    r_norm = r.dot(r);

    // cout << "Starting Levenberg-Marquardt algorithm\n";
    //cout << "P_v = [";
    //for (int i = 0; i < 8; i++) {
    //    cout << to_string(P_v_(i));
    //    if (i < 7) {
    //        cout << ", ";
    //    } else {
    //        cout << "]\n";
    //    }
    //}

    //return false;


    // Levenberg-Marquardt algorithm
    while(true) {

        // cout << "At iteration " << to_string(cnt) << "\n";

        cnt += 1;

        fdx = (this->*fd)(x);

        // cout << "x = [";
        //for (int i = 0; i < 4; i++) {
        //    cout << to_string(x(i));
        //    if (i < 3) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}
        //cout << "fx = [";
        //for (int i = 0; i < 8; i++) {
        //    cout << to_string(fx(i));
        //    if (i < 7) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}
        //cout << "r = [";
        //for (int i = 0; i < 8; i++) {
        //    cout << to_string(r(i));
        //    if (i < 7) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "r_norm = " << to_string(r_norm) << "\n";
        
        bool lma_success = false;
        lambda = 0;
        fdx_T = fdx.transpose();
        JTJ = fdx_T * fdx;
        b = fdx_T*r;

        //cout << "fdx = [\n";
        //for (int i = 0; i < 8; i++) {
        //    cout << "\t";
        //    for (int j = 0; j < 4; j++) {
        //        cout << to_string(fdx(i,j));
        //        if (j < 3) {
        //            cout << ", ";
        //        }
        //    }
        //    if (i < 7) {
        //        cout << ",\n";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "fdx_T = [\n";
        //for (int i = 0; i < 5; i++) {
        //    cout << "\t";
        //    for (int j = 0; j < 8; j++) {
        //        cout << to_string(fdx_T(i,j));
        //        if (j < 4) {
        //            cout << ", ";
        //        }
        //    }
        //    if (i < 4) {
        //        cout << ",\n";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "JTJ = [\n";
        //for (int i = 0; i < 5; i++) {
        //    cout << "\t";
        //    for (int j = 0; j < 5; j++) {
        //        cout << to_string(JTJ(i,j));
        //        if (j < 4) {
        //            cout << ", ";
        //        }
        //    }
        //    if (i < 4) {
        //        cout << ",\n";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "b = [";
        //for (int i = 0; i < 4; i++) {
        //    cout << b(i);
        //    if (i < 3) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        for (int i = 0; i < max_weighting_iterations_; i++) {

            A = JTJ + lambda * eye;
            d = A.inverse() * b;
            x_tmp = x + d;

            fx = (this->*f)(x_tmp);

            r_tmp = P_v_ - fx;
            r_tmp_norm = r_tmp.dot(r_tmp);

            if (r_tmp_norm < r_norm) {

                //cout << "LMA success after " << to_string(i+1) << " iterations\n";
                //cout << "lambda = " << to_string(lambda) << "\n";
                //cout << "A = [\n";
                //for (int i = 0; i < 4; i++) {
                //    cout << "\t";
                //    for (int j = 0; j < 4; j++) {
                //        cout << to_string(A(i,j));
                //        if (j < 3) {
                //            cout << ", ";
                //        }
                //    }
                //    if (i < 3) {
                //        cout << ",\n";
                //    } else {
                //        cout << "]\n";
                //    }
                //}
                //cout << "d = [";
                //for (int i = 0; i < 4; i++) {
                //    cout << to_string(d(i));
                //    if(i < 3) {
                //        cout << ", ";
                //    } else {
                //        cout << "]\n";
                //    }
                //}
                //cout << "r_tmp_norm = " << to_string(r_tmp_norm) << "\n" << endl;

                x = x_tmp;
                r = r_tmp;
                r_norm = r_tmp_norm;
                lma_success = true;
                break;

            }

            if (lambda == 0) {
                lambda = lambda_first_step_;
            } else {
                lambda *= lambda_increment_factor_;
            }

        }

        if (!lma_success) {

            //cout << "LMA did not suceed\n" << endl;
            //cout << "lambda = " << to_string(lambda) << "\n";
            //cout << "r_norm = " << to_string(r_norm) << "\n" << endl;
            break;

        }

        //cout << "x = [";
        //for (int i = 0; i < 4; i++) {
        //    cout << to_string(x(i));
        //    if (i < 3) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        if (d.norm() <= step_norm_success_threshold_) {

            //cout << "Finished!\n" << endl;

            success = true;
            break;

        }

        if (cnt > max_LMA_iterations_) {

            //cout << "Reached maximum number of iterations, aborting" << endl;

            break;

        }

        //if (abs(x(0)) > I_max_) {

        //    cout << "I exceeds boundary value, aborting" << endl;
        //    break;

        //}

        vector_t p1(0,x(0),x(1));
        vector_t p2(0,x(2),x(3));

        //if (p1.norm() > cable_distance_max_ && p2.norm() > cable_distance_max_) {

        //    //cout << "p1 or p2 exceeds maximum distance, aborting" << endl;
        //    break;

        //}

        //cout << endl;
        //cout << endl;
        //cout << endl;
    }

    return success;
}

Eigen::MatrixXd PowerlinePositionsComputerNode::J(const Eigen::VectorXd x) {

    using namespace Eigen;
    using namespace std;

    static MatrixXd J(8,4);
    static double I;
    static double P1_D_py;
    static double P1_D_pz;
    static double P2_D_py;
    static double P2_D_pz;
    static const double pi = M_PI;
    static const double u0 = mu_0_;
    static const double T_per_LSB = T_per_LSB_;
    static double p1y2;
    static double p1z2;
    static double p2y2;
    static double p2z2;
    static double p1y2_p1z2;
    static double p2y2_p2z2;
    static double p1y2_p1z2_2;
    static double p2y2_p2z2_2;
    static double P_D_Mja_p2_1;
    static double P_D_Mja_p2_2;
    static double P_D_Mja_p2_3;
    static double P_D_Mja_p2_4;
    static double P_D_Mja_p3_1;
    static double P_D_Mja_p3_2;
    static double P_D_Mja_p3_3;
    static double P_D_Mja_p3_4;

    P_D_Mja_p2_1 = P_D_Mja_p_[0](1);
    P_D_Mja_p2_2 = P_D_Mja_p_[1](1);
    P_D_Mja_p2_3 = P_D_Mja_p_[2](1);
    P_D_Mja_p2_4 = P_D_Mja_p_[3](1);
    P_D_Mja_p3_1 = P_D_Mja_p_[0](2);
    P_D_Mja_p3_2 = P_D_Mja_p_[1](2);
    P_D_Mja_p3_3 = P_D_Mja_p_[2](2);
    P_D_Mja_p3_4 = P_D_Mja_p_[3](2);

    //I = x(0);
    I = I_;
    P1_D_py = x(0);
    P1_D_pz = x(1);
    P2_D_py = x(2);
    P2_D_pz = x(3);

    //cout << "Computing jacobian\n";
    //cout << "I = " << to_string(I) << "\n";
    //cout << "P1_D_py = " << to_string(P1_D_py) << "\n";
    //cout << "P1_D_pz = " << to_string(P1_D_pz) << "\n";
    //cout << "P2_D_py = " << to_string(P2_D_py) << "\n";
    //cout << "P2_D_pz = " << to_string(P2_D_pz) << "\n";

    p1y2 = P1_D_py * P1_D_py;
    p1z2 = P1_D_pz * P1_D_pz;
    p2y2 = P2_D_py * P2_D_py;
    p2z2 = P2_D_pz * P2_D_pz;
    p1y2_p1z2 = p1y2 + p1z2;
    p2y2_p2z2 = p2y2 + p2z2;
    p1y2_p1z2_2 = p1y2_p1z2 * p1y2_p1z2;
    p2y2_p2z2_2 = p2y2_p2z2 * p2y2_p2z2;

    //J(0,0) = ((u0*(P1_D_pz - P_D_Mja_p3_1))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2))) - (u0*(P2_D_pz - P_D_Mja_p3_1))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2))))/T_per_LSB;
    J(0,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_1)*(2*P1_D_py - 2*P_D_Mja_p2_1))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2));
    J(0,1) = ((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_1)*(2*P1_D_pz - 2*P_D_Mja_p3_1))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2)))/T_per_LSB;
    J(0,2) = (I*u0*(P2_D_pz - P_D_Mja_p3_1)*(2*P2_D_py - 2*P_D_Mja_p2_1))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2),2));
    J(0,3) = -((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2))) - (I*u0*(P2_D_pz - P_D_Mja_p3_1)*(2*P2_D_pz - 2*P_D_Mja_p3_1))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2),2)))/T_per_LSB;
        
    //J(1,0) = -((u0*(P1_D_py - P_D_Mja_p2_1))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2))) - (u0*(P2_D_py - P_D_Mja_p2_1))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2))))/T_per_LSB;
    J(1,0) = -((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2))) - (I*u0*(P1_D_py - P_D_Mja_p2_1)*(2*P1_D_py - 2*P_D_Mja_p2_1))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2)))/T_per_LSB;
    J(1,1) = (I*u0*(P1_D_py - P_D_Mja_p2_1)*(2*P1_D_pz - 2*P_D_Mja_p3_1))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2));
    J(1,2) = ((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2))) - (I*u0*(P2_D_py - P_D_Mja_p2_1)*(2*P2_D_py - 2*P_D_Mja_p2_1))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2),2)))/T_per_LSB;
    J(1,3) = -(I*u0*(P2_D_py - P_D_Mja_p2_1)*(2*P2_D_pz - 2*P_D_Mja_p3_1))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_1,2) + pow(P2_D_pz - P_D_Mja_p3_1,2),2));

    //J(2,0) = ((u0*(P1_D_pz - P_D_Mja_p3_2))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2))) - (u0*(P2_D_pz - P_D_Mja_p3_2))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2))))/T_per_LSB;
    J(2,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_2)*(2*P1_D_py - 2*P_D_Mja_p2_2))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2));
    J(2,1) = ((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_2)*(2*P1_D_pz - 2*P_D_Mja_p3_2))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2)))/T_per_LSB;
    J(2,2) = (I*u0*(P2_D_pz - P_D_Mja_p3_2)*(2*P2_D_py - 2*P_D_Mja_p2_2))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2),2));
    J(2,3) = -((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2))) - (I*u0*(P2_D_pz - P_D_Mja_p3_2)*(2*P2_D_pz - 2*P_D_Mja_p3_2))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2),2)))/T_per_LSB;

    //J(3,0) = -((u0*(P1_D_py - P_D_Mja_p2_2))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2))) - (u0*(P2_D_py - P_D_Mja_p2_2))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2))))/T_per_LSB;
    J(3,0) = -((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2))) - (I*u0*(P1_D_py - P_D_Mja_p2_2)*(2*P1_D_py - 2*P_D_Mja_p2_2))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2)))/T_per_LSB;
    J(3,1) = (I*u0*(P1_D_py - P_D_Mja_p2_2)*(2*P1_D_pz - 2*P_D_Mja_p3_2))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2));
    J(3,2) = ((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2))) - (I*u0*(P2_D_py - P_D_Mja_p2_2)*(2*P2_D_py - 2*P_D_Mja_p2_2))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2),2)))/T_per_LSB;
    J(3,3) = -(I*u0*(P2_D_py - P_D_Mja_p2_2)*(2*P2_D_pz - 2*P_D_Mja_p3_2))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_2,2) + pow(P2_D_pz - P_D_Mja_p3_2,2),2));

    //J(4,0) = ((u0*(P1_D_pz - P_D_Mja_p3_3))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2))) - (u0*(P2_D_pz - P_D_Mja_p3_3))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2))))/T_per_LSB;
    J(4,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_3)*(2*P1_D_py - 2*P_D_Mja_p2_3))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2));
    J(4,1) = ((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_3)*(2*P1_D_pz - 2*P_D_Mja_p3_3))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2)))/T_per_LSB;
    J(4,2) = (I*u0*(P2_D_pz - P_D_Mja_p3_3)*(2*P2_D_py - 2*P_D_Mja_p2_3))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2),2));
    J(4,3) = -((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2))) - (I*u0*(P2_D_pz - P_D_Mja_p3_3)*(2*P2_D_pz - 2*P_D_Mja_p3_3))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2),2)))/T_per_LSB;

    //J(5,0) = -((u0*(P1_D_py - P_D_Mja_p2_3))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2))) - (u0*(P2_D_py - P_D_Mja_p2_3))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2))))/T_per_LSB;
    J(5,0) = -((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2))) - (I*u0*(P1_D_py - P_D_Mja_p2_3)*(2*P1_D_py - 2*P_D_Mja_p2_3))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2)))/T_per_LSB;
    J(5,1) = (I*u0*(P1_D_py - P_D_Mja_p2_3)*(2*P1_D_pz - 2*P_D_Mja_p3_3))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2));
    J(5,2) = ((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2))) - (I*u0*(P2_D_py - P_D_Mja_p2_3)*(2*P2_D_py - 2*P_D_Mja_p2_3))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2),2)))/T_per_LSB;
    J(5,3) = -(I*u0*(P2_D_py - P_D_Mja_p2_3)*(2*P2_D_pz - 2*P_D_Mja_p3_3))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_3,2) + pow(P2_D_pz - P_D_Mja_p3_3,2),2));

    //J(6,0) = ((u0*(P1_D_pz - P_D_Mja_p3_4))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2))) - (u0*(P2_D_pz - P_D_Mja_p3_4))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2))))/T_per_LSB;
    J(6,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_4)*(2*P1_D_py - 2*P_D_Mja_p2_4))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2));
    J(6,1) = ((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_4)*(2*P1_D_pz - 2*P_D_Mja_p3_4))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2)))/T_per_LSB;
    J(6,2) = (I*u0*(P2_D_pz - P_D_Mja_p3_4)*(2*P2_D_py - 2*P_D_Mja_p2_4))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2),2));
    J(6,3) = -((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2))) - (I*u0*(P2_D_pz - P_D_Mja_p3_4)*(2*P2_D_pz - 2*P_D_Mja_p3_4))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2),2)))/T_per_LSB;

    //J(7,0) = -((u0*(P1_D_py - P_D_Mja_p2_4))/(2*pi*(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2))) - (u0*(P2_D_py - P_D_Mja_p2_4))/(2*pi*(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2))))/T_per_LSB;
    J(7,0) = -((I*u0)/(2*pi*(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2))) - (I*u0*(P1_D_py - P_D_Mja_p2_4)*(2*P1_D_py - 2*P_D_Mja_p2_4))/(2*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2)))/T_per_LSB;
    J(7,1) = (I*u0*(P1_D_py - P_D_Mja_p2_4)*(2*P1_D_pz - 2*P_D_Mja_p3_4))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2));
    J(7,2) = ((I*u0)/(2*pi*(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2))) - (I*u0*(P2_D_py - P_D_Mja_p2_4)*(2*P2_D_py - 2*P_D_Mja_p2_4))/(2*pi*pow(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2),2)))/T_per_LSB;
    J(7,3) = -(I*u0*(P2_D_py - P_D_Mja_p2_4)*(2*P2_D_pz - 2*P_D_Mja_p3_4))/(2*T_per_LSB*pi*pow(pow(P2_D_py - P_D_Mja_p2_4,2) + pow(P2_D_pz - P_D_Mja_p3_4,2),2));



    //J(0,0) = ((P1_D_pz*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_pz*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(0,1) = -(I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(0,2) = ((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1z2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(0,3) = (I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);
    //J(0,4) = -((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2z2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;

    //J(1,0) = -((P1_D_py*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_py*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(1,1) = -((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1y2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(1,2) = (I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(1,3) = ((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2y2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;
    //J(1,4) = -(I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);

    //J(2,0) = ((P1_D_pz*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_pz*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(2,1) = -(I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(2,2) = ((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1z2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(2,3) = (I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);
    //J(2,4) = -((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2z2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;

    //J(3,0) = -((P1_D_py*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_py*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(3,1) = -((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1y2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(3,2) = (I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(3,3) = ((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2y2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;
    //J(3,4) = -(I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);

    //J(4,0) = ((P1_D_pz*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_pz*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(4,1) = -(I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(4,2) = ((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1z2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(4,3) = (I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);
    //J(4,4) = -((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2z2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;

    //J(5,0) = -((P1_D_py*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_py*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(5,1) = -((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1y2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(5,2) = (I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(5,3) = ((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2y2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;
    //J(5,4) = -(I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);

    //J(6,0) = ((P1_D_pz*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_pz*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(6,1) = -(I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(6,2) = ((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1z2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(6,3) = (I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);
    //J(6,4) = -((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2z2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;

    //J(7,0) = -((P1_D_py*u0)/(2*pi*(p1y2_p1z2)) - (P2_D_py*u0)/(2*pi*(p2y2_p2z2)))/T_per_LSB;
    //J(7,1) = -((I*u0)/(2*pi*(p1y2_p1z2)) - (I*p1y2*u0)/(pi*p1y2_p1z2_2))/T_per_LSB;
    //J(7,2) = (I*P1_D_py*P1_D_pz*u0)/(T_per_LSB*pi*p1y2_p1z2_2);
    //J(7,3) = ((I*u0)/(2*pi*(p2y2_p2z2)) - (I*p2y2*u0)/(pi*p2y2_p2z2_2))/T_per_LSB;
    //J(7,4) = -(I*P2_D_py*P2_D_pz*u0)/(T_per_LSB*pi*p2y2_p2z2_2);

    return J;

}

Eigen::VectorXd PowerlinePositionsComputerNode::B(const Eigen::VectorXd x) {

    using namespace Eigen;
    using namespace std;

	static double I;
    static Vector2d p1;
    static Vector2d p2;
    static Vector2d P_B_j[4];
    static Vector2d P1_B_j[4];
    static Vector2d P2_B_j[4];
    static double norm_vals_1[4];
    static double norm_vals_2[4];
    static double C;
    static VectorXd b(8);

    //cout << "Computing B\n";

    //for (int i = 0; i < 4; i++) {
    //    cout << "P_D_Mja_p[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(P_D_Mja_p_[i](j));
    //        if (j < 2) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}


    //I = x(0);
    I = I_;
    p1(0) = x(0);
    p1(1) = x(1);
    p2(0) = x(2);
    p2(1) = x(3);

    //cout << "I = " << to_string(I) << "\n";
    //cout << "p1 = [" << to_string(p1(0)) << ", " << to_string(p1(1)) << "]\n";
    //cout << "p1 = [" << to_string(p2(0)) << ", " << to_string(p2(1)) << "]\n";

    C = I * C_multiplier_;

    //cout << "C = " << to_string(C) << "\n";

    for (int i = 0; i < 4; i++) {
        
        norm_vals_1[i] = pow(p1(0) - P_D_Mja_p_[i](1), 2) + pow(p1(1) - P_D_Mja_p_[i](2), 2);
        norm_vals_2[i] = pow(p2(0) - P_D_Mja_p_[i](1), 2) + pow(p2(1) - P_D_Mja_p_[i](2), 2);

        P1_B_j[i](0) = C * (p1(1) - P_D_Mja_p_[i](2)) / norm_vals_1[i];
        P1_B_j[i](1) = C * (-p1(0) + P_D_Mja_p_[i](1)) / norm_vals_1[i];
        
        P2_B_j[i](0) = C * (p2(1) - P_D_Mja_p_[i](2)) / norm_vals_2[i];
        P2_B_j[i](1) = C * (-p2(0) + P_D_Mja_p_[i](1)) / norm_vals_2[i];

        P_B_j[i] = (P1_B_j[i] - P2_B_j[i]) / T_per_LSB_;

        b(i*2) = P_B_j[i](0);
        b(i*2+1) = P_B_j[i](1);

    }

    //cout << "norm_vals_1 = [";
    //for (int i = 0; i < 4; i++) {
    //    cout << to_string(norm_vals_1[i]);
    //    if (i < 3) {
    //        cout << ", ";
    //    }
    //    else {
    //        cout << "]\n";
    //    }
    //}

    //cout << "norm_vals_2 = [";
    //for (int i = 0; i < 4; i++) {
    //    cout << to_string(norm_vals_2[i]);
    //    if (i < 3) {
    //        cout << ", ";
    //    }
    //    else {
    //        cout << "]\n";
    //    }
    //}

    //for (int i = 0; i < 4; i++) {
    //    cout << "P1_B_j[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 2; j++) {
    //        cout << to_string(P1_B_j[i](j));
    //        if (j < 1) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //for (int i = 0; i < 4; i++) {
    //    cout << "P2_B_j[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 2; j++) {
    //        cout << to_string(P2_B_j[i](j));
    //        if (j < 1) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //cout << "b = [";
    //for (int i = 0; i < 8; i++) {
    //    cout << to_string(b(i));
    //    if (i < 7) {
    //        cout << ", ";
    //    } else {
    //        cout << "]\n";
    //    }
    //}

    return b;
    
}

void PowerlinePositionsComputerNode::singleCableNLLSComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors) {

    using namespace Eigen;

    static VectorXd x(2);

    rotation_matrix_t D_P_R = quatToMat(pl_dir_quat_hold_);

    rotation_matrix_t D_P_R_T = D_P_R.transpose();

    vector_t P_vj[4];

    //for (int i = 0; i < 4; i++) {
    //    cout << "v_drone_to_mags[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(v_drone_to_mags_[i](j));
    //        if (j < 2) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //for (int i = 0; i < 4; i++) {
    //    cout << "v_proj_mags_[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(v_proj_mags_[i](j));
    //        if (j < 2) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //cout << "D_P_R = [\n";
    //for (int i = 0; i < 3; i++) {
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(D_P_R(i,j));
    //        if (i == 2 and j == 2) {
    //            cout << "]\n";
    //        } else {
    //            cout << ",";
    //        }
    //    }
    //    cout << "\n";
    //}

    for (int i = 0; i < 4; i++) {

        P_D_Mja_p_[i] = D_P_R_T * v_proj_mags_[i];

        vector_t D_vj = D_vj_vecs_[i];

        // cout << "D_vj(:," << to_string(i+1) << ") = [" << to_string(D_vj(0)) << " " << to_string(D_vj(1)) << " " << to_string(D_vj(2)) << "];\n";

        P_vj[i] = D_P_R * D_vj;

        P_v_(i*2) = P_vj[i](1);
        P_v_(i*2+1) = P_vj[i](2);

    }
    // cout << endl;

    vector_t p1,p2;
    p_est_mutex_.lock(); {
        p1 = p1_;
        p2 = p2_;
    } p_est_mutex_.unlock();

    vector_t P_p1_ = D_P_R_T * p1_;


    //x(0) = P_p1_(1);
    //x(1) = P_p1_(2);

    x(0) = P_p1_(1);
    x(1) = P_p1_(2);

    bool success = levenbergMarquardtSingleCable(x);

    if (!success) {
        I_ = -I_;
        x(0) = P_p1_(1);
        x(1) = P_p1_(2);

        //x(0) = -120.;
        //x(0) = P_p2_(1);
        //x(1) = P_p2_(2);
        //x(2) = P_p1_(1);
        //x(3) = P_p1_(2);
        //if (P_p1_.norm() < cable_distance_max_) {

        //    x(2) = P_p1_(1);
        //    x(3) = P_p1_(2);

        //} else {

        //    x(2) = 1;
        //    x(3) = 1;

        //}

        //if (P_p2_.norm() < cable_distance_max_) {

        //    x(0) = P_p2_(1);
        //    x(1) = P_p2_(2);

        //} else {

        //    x(0) = -1;
        //    x(1) = 1;

        //}

        success = levenbergMarquardtSingleCable(x);
    }
    //bool success = false;

    //if (!success) {

    //    std::vector<geometry_msgs::msg::Pose> poses;
    //    pl_poses_ = poses;
    //    pl_current_ = 0;

    //    return;

    //}

    // I_ = x(0);
    P_p1_(0) = 0;
    P_p1_(1) = x(0);
    P_p1_(2) = x(1);

    std::vector<geometry_msgs::msg::Pose> poses;

    geometry_msgs::msg::Pose pose1;

    publish_p1_ = false;
    publish_p2_ = false;

    if (success) {

        vector_t p1_tmp = D_P_R * P_p1_;

        vector_t p1_tmp_m_p1 = p1_tmp - p1;
        vector_t p1_tmp_m_p2 = p1_tmp - p2;

        double p1_tmp_m_p1_norm = p1_tmp_m_p1.norm();
        double p1_tmp_m_p2_norm = p1_tmp_m_p2.norm();

        int index;

        if (p1_tmp_m_p1_norm < p1_tmp_m_p2_norm) {
            
            index = 0;

        } else {

            index = 1;

        }

        if (P_p1_.norm() < cable_distance_max_) {

            if (index == 0) {

                p1 = p1_tmp;
                publish_p1_ = true;

            } else {

                p2 = p1_tmp;
                publish_p2_ = true;

            }
        }

        if (publish_p1_) {

            pose1.orientation.w = pl_dir_quat_hold_(0);
            pose1.orientation.x = pl_dir_quat_hold_(1);
            pose1.orientation.y = pl_dir_quat_hold_(2);
            pose1.orientation.z = pl_dir_quat_hold_(3);
            pose1.position.x = p1(0);
            pose1.position.y = p1(1);
            pose1.position.z = p1(2);

            poses.push_back(pose1);

            p_est_mutex_.lock(); {
                p1_ = p1;
            } p_est_mutex_.unlock();

        }
    }

    pl_poses_ = poses;
    pl_current_ = I_;

}

bool PowerlinePositionsComputerNode::levenbergMarquardtSingleCable(Eigen::VectorXd &x) {

    using namespace Eigen;
    using namespace std;

    static VectorXd d(2);
    static VectorXd fx(8);
    static VectorXd fx_tmp(8);
    static MatrixXd fdx(8,2);
    static MatrixXd fdx_T(8,2);
    static MatrixXd JTJ(2,2);
    static VectorXd r(8);
    static VectorXd r_tmp(8);
    static double r_norm;
    static double r_tmp_norm;
    static VectorXd x_tmp(2);
    static double lambda = 0;
    static MatrixXd A(2,2);
    static VectorXd b(2);
    static MatrixXd eye = Eigen::MatrixXd::Identity(2,2);

    static VectorXd (PowerlinePositionsComputerNode::*f)(VectorXd x) = &PowerlinePositionsComputerNode::BSingleCable;
    static MatrixXd (PowerlinePositionsComputerNode::*fd)(VectorXd x) = &PowerlinePositionsComputerNode::JSingleCable;

    int cnt = 0;
    bool success = false;

    lambda = 0;
    x_tmp = x;
    fx = (this->*f)(x);
    r = P_v_ - fx;
    r_tmp = r;
    r_norm = r.dot(r);

    // cout << "Starting Levenberg-Marquardt algorithm\n";
    //cout << "P_v = [";
    //for (int i = 0; i < 8; i++) {
    //    cout << to_string(P_v_(i));
    //    if (i < 7) {
    //        cout << ", ";
    //    } else {
    //        cout << "]\n";
    //    }
    //}

    //return false;


    // Levenberg-Marquardt algorithm
    while(true) {

        // cout << "At iteration " << to_string(cnt) << "\n";

        cnt += 1;

        fdx = (this->*fd)(x);

        // cout << "x = [";
        //for (int i = 0; i < 4; i++) {
        //    cout << to_string(x(i));
        //    if (i < 3) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}
        //cout << "fx = [";
        //for (int i = 0; i < 8; i++) {
        //    cout << to_string(fx(i));
        //    if (i < 7) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}
        //cout << "r = [";
        //for (int i = 0; i < 8; i++) {
        //    cout << to_string(r(i));
        //    if (i < 7) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "r_norm = " << to_string(r_norm) << "\n";
        
        bool lma_success = false;
        lambda = 0;
        fdx_T = fdx.transpose();
        JTJ = fdx_T * fdx;
        b = fdx_T*r;

        //cout << "fdx = [\n";
        //for (int i = 0; i < 8; i++) {
        //    cout << "\t";
        //    for (int j = 0; j < 4; j++) {
        //        cout << to_string(fdx(i,j));
        //        if (j < 3) {
        //            cout << ", ";
        //        }
        //    }
        //    if (i < 7) {
        //        cout << ",\n";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "fdx_T = [\n";
        //for (int i = 0; i < 5; i++) {
        //    cout << "\t";
        //    for (int j = 0; j < 8; j++) {
        //        cout << to_string(fdx_T(i,j));
        //        if (j < 4) {
        //            cout << ", ";
        //        }
        //    }
        //    if (i < 4) {
        //        cout << ",\n";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "JTJ = [\n";
        //for (int i = 0; i < 5; i++) {
        //    cout << "\t";
        //    for (int j = 0; j < 5; j++) {
        //        cout << to_string(JTJ(i,j));
        //        if (j < 4) {
        //            cout << ", ";
        //        }
        //    }
        //    if (i < 4) {
        //        cout << ",\n";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        //cout << "b = [";
        //for (int i = 0; i < 4; i++) {
        //    cout << b(i);
        //    if (i < 3) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        for (int i = 0; i < max_weighting_iterations_; i++) {

            A = JTJ + lambda * eye;
            d = A.inverse() * b;
            x_tmp = x + d;

            fx = (this->*f)(x_tmp);

            r_tmp = P_v_ - fx;
            r_tmp_norm = r_tmp.dot(r_tmp);

            if (r_tmp_norm < r_norm) {

                //cout << "LMA success after " << to_string(i+1) << " iterations\n";
                //cout << "lambda = " << to_string(lambda) << "\n";
                //cout << "A = [\n";
                //for (int i = 0; i < 4; i++) {
                //    cout << "\t";
                //    for (int j = 0; j < 4; j++) {
                //        cout << to_string(A(i,j));
                //        if (j < 3) {
                //            cout << ", ";
                //        }
                //    }
                //    if (i < 3) {
                //        cout << ",\n";
                //    } else {
                //        cout << "]\n";
                //    }
                //}
                //cout << "d = [";
                //for (int i = 0; i < 4; i++) {
                //    cout << to_string(d(i));
                //    if(i < 3) {
                //        cout << ", ";
                //    } else {
                //        cout << "]\n";
                //    }
                //}
                //cout << "r_tmp_norm = " << to_string(r_tmp_norm) << "\n" << endl;

                x = x_tmp;
                r = r_tmp;
                r_norm = r_tmp_norm;
                lma_success = true;
                break;

            }

            if (lambda == 0) {
                lambda = lambda_first_step_;
            } else {
                lambda *= lambda_increment_factor_;
            }

        }

        if (!lma_success) {

            //cout << "LMA did not suceed\n" << endl;
            //cout << "lambda = " << to_string(lambda) << "\n";
            //cout << "r_norm = " << to_string(r_norm) << "\n" << endl;
            break;

        }

        //cout << "x = [";
        //for (int i = 0; i < 4; i++) {
        //    cout << to_string(x(i));
        //    if (i < 3) {
        //        cout << ", ";
        //    } else {
        //        cout << "]\n";
        //    }
        //}

        if (d.norm() <= step_norm_success_threshold_) {

            //cout << "Finished!\n" << endl;

            success = true;
            break;

        }

        if (cnt > max_LMA_iterations_) {

            //cout << "Reached maximum number of iterations, aborting" << endl;

            break;

        }

        //if (abs(x(0)) > I_max_) {

        //    cout << "I exceeds boundary value, aborting" << endl;
        //    break;

        //}

        vector_t p1(0,x(0),x(1));

        //if (p1.norm() > cable_distance_max_ && p2.norm() > cable_distance_max_) {

        //    //cout << "p1 or p2 exceeds maximum distance, aborting" << endl;
        //    break;

        //}

        //cout << endl;
        //cout << endl;
        //cout << endl;
    }

    return success;
}

Eigen::MatrixXd PowerlinePositionsComputerNode::JSingleCable(const Eigen::VectorXd x) {

    using namespace Eigen;
    using namespace std;

    static MatrixXd J(8,2);
    static double I;
    static double P1_D_py;
    static double P1_D_pz;
    static const double pi = M_PI;
    static const double u0 = mu_0_;
    static const double T_per_LSB = T_per_LSB_;
    static double p1y2;
    static double p1z2;
    static double p2y2;
    static double p2z2;
    static double p1y2_p1z2;
    static double p2y2_p2z2;
    static double p1y2_p1z2_2;
    static double p2y2_p2z2_2;
    static double P_D_Mja_p2_1;
    static double P_D_Mja_p2_2;
    static double P_D_Mja_p2_3;
    static double P_D_Mja_p2_4;
    static double P_D_Mja_p3_1;
    static double P_D_Mja_p3_2;
    static double P_D_Mja_p3_3;
    static double P_D_Mja_p3_4;

    P_D_Mja_p2_1 = P_D_Mja_p_[0](1);
    P_D_Mja_p2_2 = P_D_Mja_p_[1](1);
    P_D_Mja_p2_3 = P_D_Mja_p_[2](1);
    P_D_Mja_p2_4 = P_D_Mja_p_[3](1);
    P_D_Mja_p3_1 = P_D_Mja_p_[0](2);
    P_D_Mja_p3_2 = P_D_Mja_p_[1](2);
    P_D_Mja_p3_3 = P_D_Mja_p_[2](2);
    P_D_Mja_p3_4 = P_D_Mja_p_[3](2);

    //I = x(0);
    I = I_;
    P1_D_py = x(0);
    P1_D_pz = x(1);

    //cout << "Computing jacobian\n";
    //cout << "I = " << to_string(I) << "\n";
    //cout << "P1_D_py = " << to_string(P1_D_py) << "\n";
    //cout << "P1_D_pz = " << to_string(P1_D_pz) << "\n";
    //cout << "P2_D_py = " << to_string(P2_D_py) << "\n";
    //cout << "P2_D_pz = " << to_string(P2_D_pz) << "\n";

    J(0,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_1)*(2*P1_D_py - 2*P_D_Mja_p2_1))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2));
    J(0,1) = (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_1)*(2*P1_D_pz - 2*P_D_Mja_p3_1))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2));
    J(1,0) = (I*u0*(P1_D_py - P_D_Mja_p2_1)*(2*P1_D_py - 2*P_D_Mja_p2_1))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2)) - (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2)));
    J(1,1) = (I*u0*(P1_D_py - P_D_Mja_p2_1)*(2*P1_D_pz - 2*P_D_Mja_p3_1))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_1,2) + pow(P1_D_pz - P_D_Mja_p3_1,2),2));
    J(2,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_2)*(2*P1_D_py - 2*P_D_Mja_p2_2))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2));
    J(2,1) = (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_2)*(2*P1_D_pz - 2*P_D_Mja_p3_2))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2));
    J(3,0) = (I*u0*(P1_D_py - P_D_Mja_p2_2)*(2*P1_D_py - 2*P_D_Mja_p2_2))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2)) - (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2)));
    J(3,1) = (I*u0*(P1_D_py - P_D_Mja_p2_2)*(2*P1_D_pz - 2*P_D_Mja_p3_2))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_2,2) + pow(P1_D_pz - P_D_Mja_p3_2,2),2));
    J(4,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_3)*(2*P1_D_py - 2*P_D_Mja_p2_3))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2));
    J(4,1) = (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_3)*(2*P1_D_pz - 2*P_D_Mja_p3_3))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2));
    J(5,0) = (I*u0*(P1_D_py - P_D_Mja_p2_3)*(2*P1_D_py - 2*P_D_Mja_p2_3))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2)) - (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2)));
    J(5,1) = (I*u0*(P1_D_py - P_D_Mja_p2_3)*(2*P1_D_pz - 2*P_D_Mja_p3_3))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_3,2) + pow(P1_D_pz - P_D_Mja_p3_3,2),2));
    J(6,0) = -(I*u0*(P1_D_pz - P_D_Mja_p3_4)*(2*P1_D_py - 2*P_D_Mja_p2_4))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2));
    J(6,1) = (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2))) - (I*u0*(P1_D_pz - P_D_Mja_p3_4)*(2*P1_D_pz - 2*P_D_Mja_p3_4))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2));
    J(7,0) = (I*u0*(P1_D_py - P_D_Mja_p2_4)*(2*P1_D_py - 2*P_D_Mja_p2_4))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2)) - (I*u0)/(2*T_per_LSB*pi*(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2)));
    J(7,1) = (I*u0*(P1_D_py - P_D_Mja_p2_4)*(2*P1_D_pz - 2*P_D_Mja_p3_4))/(2*T_per_LSB*pi*pow(pow(P1_D_py - P_D_Mja_p2_4,2) + pow(P1_D_pz - P_D_Mja_p3_4,2),2));

    return J;

}

Eigen::VectorXd PowerlinePositionsComputerNode::BSingleCable(const Eigen::VectorXd x) {

    using namespace Eigen;
    using namespace std;

	static double I;
    static Vector2d p1;
    static Vector2d p2;
    static Vector2d P_B_j[4];
    static Vector2d P1_B_j[4];
    static double norm_vals_1[4];
    static double C;
    static VectorXd b(8);

    //cout << "Computing B\n";

    //for (int i = 0; i < 4; i++) {
    //    cout << "P_D_Mja_p[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 3; j++) {
    //        cout << to_string(P_D_Mja_p_[i](j));
    //        if (j < 2) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}


    //I = x(0);
    I = I_;
    p1(0) = x(0);
    p1(1) = x(1);

    //cout << "I = " << to_string(I) << "\n";
    //cout << "p1 = [" << to_string(p1(0)) << ", " << to_string(p1(1)) << "]\n";
    //cout << "p1 = [" << to_string(p2(0)) << ", " << to_string(p2(1)) << "]\n";

    C = I * C_multiplier_;

    //cout << "C = " << to_string(C) << "\n";

    for (int i = 0; i < 4; i++) {
        
        norm_vals_1[i] = pow(p1(0) - P_D_Mja_p_[i](1), 2) + pow(p1(1) - P_D_Mja_p_[i](2), 2);

        P1_B_j[i](0) = C * (p1(1) - P_D_Mja_p_[i](2)) / norm_vals_1[i];
        P1_B_j[i](1) = C * (-p1(0) + P_D_Mja_p_[i](1)) / norm_vals_1[i];
        
        P_B_j[i] = (P1_B_j[i]) / T_per_LSB_;

        b(i*2) = P_B_j[i](0);
        b(i*2+1) = P_B_j[i](1);

    }

    //cout << "norm_vals_1 = [";
    //for (int i = 0; i < 4; i++) {
    //    cout << to_string(norm_vals_1[i]);
    //    if (i < 3) {
    //        cout << ", ";
    //    }
    //    else {
    //        cout << "]\n";
    //    }
    //}

    //cout << "norm_vals_2 = [";
    //for (int i = 0; i < 4; i++) {
    //    cout << to_string(norm_vals_2[i]);
    //    if (i < 3) {
    //        cout << ", ";
    //    }
    //    else {
    //        cout << "]\n";
    //    }
    //}

    //for (int i = 0; i < 4; i++) {
    //    cout << "P1_B_j[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 2; j++) {
    //        cout << to_string(P1_B_j[i](j));
    //        if (j < 1) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //for (int i = 0; i < 4; i++) {
    //    cout << "P2_B_j[" << to_string(i) << "] = [";
    //    for (int j = 0; j < 2; j++) {
    //        cout << to_string(P2_B_j[i](j));
    //        if (j < 1) {
    //            cout << ", ";
    //        } else {
    //            cout << "]\n";
    //        }
    //    }
    //}

    //cout << "b = [";
    //for (int i = 0; i < 8; i++) {
    //    cout << to_string(b(i));
    //    if (i < 7) {
    //        cout << ", ";
    //    } else {
    //        cout << "]\n";
    //    }
    //}

    return b;
    
}

void PowerlinePositionsComputerNode::publishPoseEstimationResult() {

    mag_pl_detector::msg::PowerlinePosesComputationResult result_msg;
    result_msg.header.frame_id = "drone";
    result_msg.header.stamp = this->get_clock()->now();

    result_msg.current = pl_current_;
    result_msg.poses = pl_poses_;

    pl_poses_result_raw_pub_->publish(result_msg);

    if (pl_poses_.size() > 0) {

        //std::cout << "Publishing 0" << std::endl;
        //std::cout << std::to_string(pl_poses_.size()) << std::endl;

        geometry_msgs::msg::PoseStamped msg0;
        msg0.header.frame_id = "drone";
        msg0.header.stamp = result_msg.header.stamp;

        msg0.pose = pl_poses_[0];

        if (publish_p1_) {

            pl0_pose_raw_pub_->publish(msg0);

        } else {

            pl1_pose_raw_pub_->publish(msg0);
            return;

        }

        if (pl_poses_.size() > 1) {

            // std::cout << "Publishing 1" << endl;

            geometry_msgs::msg::PoseStamped msg1;
            msg1.header.frame_id = "drone";
            msg1.header.stamp = result_msg.header.stamp;

            msg1.pose = pl_poses_[1];

            pl1_pose_raw_pub_->publish(msg1);

        }
    }
}

void PowerlinePositionsComputerNode::fetchStaticTransforms() {

	for (int i = 0; i < 4; i++) {

		geometry_msgs::msg::TransformStamped mag_tf;

		while(true) {

			try {

				mag_tf = tf_buffer_->lookupTransform("drone", mag_frame_names_[i], tf2::TimePointZero);

				break;

			} catch(tf2::TransformException & ex) {

				//RCLCPP_INFO(this->get_logger(), "Could not get mag transform, trying again...");

			}
		}

		quat_t mag_quat(
			mag_tf.transform.rotation.w,
			mag_tf.transform.rotation.x,
			mag_tf.transform.rotation.y,
			mag_tf.transform.rotation.z
		);

		R_drone_to_mags_[i] = quatToMat(mag_quat);

		v_drone_to_mags_[i](0) = mag_tf.transform.translation.x;
		v_drone_to_mags_[i](1) = mag_tf.transform.translation.y;
		v_drone_to_mags_[i](2) = mag_tf.transform.translation.z;

	}

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<PowerlinePositionsComputerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}