/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "pl_positions_computer.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

PowerlinePositionsComputerNode::PowerlinePositionsComputerNode(const std::string & node_name, const std::string & node_namespace)
                            : Node(node_name, node_namespace), optimizer_(nlopt::GN_DIRECT_L, 7) {

    this->declare_parameter<double>("I_min", 100);
    this->get_parameter("I_min", I_min_);

    this->declare_parameter<double>("I_max", 1000);
    this->get_parameter("I_max", I_max_);

    this->declare_parameter<double>("xyz_min", -10);
    this->get_parameter("xyz_min", xyz_min_);

    this->declare_parameter<double>("xyz_max", 10);
    this->get_parameter("xyz_max", xyz_max_);

    RCLCPP_INFO(this->get_logger(), "Starting %s with parameters:%sI_min: %d %sI_max: %d %sxyz_min: %d %sxyz_max: %d %s%s",
		node_name, std::endl, I_min_, std::endl, I_max_, std::endl, xyz_min_, std::endl, xyz_max_, std::endl, std::endl);

    lb_.push_back(I_min_);
    ub_.push_back(I_max_);

    for (int i = 0; i < 6; i++) {

        lb_.push_back(xyz_min_);
        ub_.push_back(xyz_max_);

    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	fetchStaticTransforms();

    mag_phasors_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasors3D>(
        "/sine_reconstructor/mag_phasors", 10,
        std::bind(&PowerlinePositionsComputerNode::magPhasorsCallback, this, std::placeholders::_1)
    );

    pl_dir_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pl_dir_estimator/powerline_direction_est", 10,
        std::bind(&PowerlinePositionsComputerNode::plDirCallback, this, std::placeholders::_1)
    );

    pl_poses_result_raw_pub_ = this->create_publisher<mag_pl_detector::msg::PowerlinePosesComputationResult>(
        "pl_poses_raw", 10
    );

    odometry_timer_ = this->create_wall_timer(
        10ms, std::bind(&PowerlinePositionsComputerNode::odometryCallback, this));
    
    quat_t q(1,0,0,0);
    drone_quat_ = q;
    pl_dir_quat_ = q;

    x_.push_back(100);

    for (int i = 0; i < 6; i++) {

        x_.push_back(0);

    }

    opt_f_ = 0;

    vectord_t v(0,0,0);
    for (int i = 0; i < 4; i++){

        opt_data_.D_Mjalpha_p[i] = v;
        opt_data_.D_V[i*3] = 0;
        opt_data_.D_V[i*3+1] = 0;
        opt_data_.D_V[i*3+2] = 0;

    }

    opt_data_.D_alpha_n_norm = v;

}

void PowerlinePositionsComputerNode::magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg) {

    std::cout << "MagPhasorsCallback" << std::endl;

    prepareOptimization(*msg);
    runOptimization();
    //publishPositions();

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

    if (odometry_mutex_.try_lock()) {

        drone_quat_ = quat;

        odometry_mutex_.unlock();

    }

}

void PowerlinePositionsComputerNode::prepareOptimization(mag_pl_detector::msg::MagneticPhasors3D msg) {

    //quat_t drone_quat;

    //odometry_mutex_.lock(); {

    //    drone_quat = drone_quat_;

    //} odometry_mutex_.unlock();

    pl_dir_mutex_.lock(); {

        pl_dir_quat_holder_ = pl_dir_quat_;

    } pl_dir_mutex_.unlock();

    std::cout << "Preparing optimization" << std::endl;

    measVecSet(msg);
    alphaSet(pl_dir_quat_holder_);
    projMagPosSet();
    opt_data_.C = mu_0_ / (2*M_PI);
    opt_data_.inverted = false;
    opt_data_.epsilon = std::numeric_limits<double>::epsilon();
    opt_data_.conversion_factor = 1/T_per_LSB_;

    optimizer_.set_min_objective(PowerlinePositionsComputerNode::F, (void *)&opt_data_);
    optimizer_.set_lower_bounds(lb_);
    optimizer_.set_upper_bounds(ub_);

}

void PowerlinePositionsComputerNode::runOptimization() {

    std::cout << "Running optimization" << std::endl;

    static bool hej = true;

    if (hej)  {
        x_[0] = 120;
        x_[1] = 0;
        x_[2] = 0;
        x_[3] = 1;
        x_[4] = 0;
        x_[5] = 3;
        x_[6] = 3;

        hej = false;
    }

    std::vector<double> grad;
    for (int i = 0; i < 7; i++) {
        grad.push_back(0);
    }

    std::cout << "SSE function value: " << std::to_string(F(x_, grad, (void *)&opt_data_)) << std::endl;

    for (int i = 0; i < 7; i++) {
        std::cout << "grad[" << std::to_string(i) << "]: " << std::to_string(grad[i]) << std::endl;
    }
    std::cout << std::endl;

    optimizer_.optimize(x_, opt_f_);


    //exit(-1);

}

void PowerlinePositionsComputerNode::publishPositions() {

    mag_pl_detector::msg::PowerlinePosesComputationResult msg;
    msg.header.frame_id = "drone";
    msg.header.stamp = this->get_clock()->now();

    msg.current = x_[0];

    geometry_msgs::msg::Pose cable1_pose;
    cable1_pose.orientation.w = pl_dir_quat_(0);
    cable1_pose.orientation.x = pl_dir_quat_(1);
    cable1_pose.orientation.y = pl_dir_quat_(2);
    cable1_pose.orientation.z = pl_dir_quat_(3);
    cable1_pose.position.x = x_[1];
    cable1_pose.position.y = x_[2];
    cable1_pose.position.z = x_[3];

    geometry_msgs::msg::Pose cable2_pose;
    cable2_pose.orientation.w = pl_dir_quat_(0);
    cable2_pose.orientation.x = pl_dir_quat_(1);
    cable2_pose.orientation.y = pl_dir_quat_(2);
    cable2_pose.orientation.z = pl_dir_quat_(3);
    cable2_pose.position.x = x_[4];
    cable2_pose.position.y = x_[5];
    cable2_pose.position.z = x_[6];

    msg.poses.push_back(cable1_pose);
    msg.poses.push_back(cable2_pose);

    pl_poses_result_raw_pub_->publish(msg);

}

void PowerlinePositionsComputerNode::measVecSet(mag_pl_detector::msg::MagneticPhasors3D msg) {

    for (int i = 0; i < 4; i++) {

        opt_data_.D_V(i*3) = msg.phasors[i].amplitudes.x;
        opt_data_.D_V(i*3+1) = msg.phasors[i].amplitudes.y;
        opt_data_.D_V(i*3+2) = msg.phasors[i].amplitudes.z;

        std::cout << "D_v" << std::to_string(i) << ": [" << std::to_string(opt_data_.D_V(i*3)) << "," << std::to_string(opt_data_.D_V(i*3+1)) << "," << std::to_string(opt_data_.D_V(i*3+2)) << "]^T" << std::endl;

    }
}

void PowerlinePositionsComputerNode::alphaSet(quat_t pl_dir_quat) {

    vector_t unit_x(1,0,0);
    
    rotation_matrix_t pl_dir_R = quatToMat(pl_dir_quat);

    vector_t dL = rotateVector(pl_dir_R, unit_x);

    dL /= dL.norm();

    opt_data_.D_alpha_n_norm(0) = (double)dL(0);
    opt_data_.D_alpha_n_norm(1) = (double)dL(1);
    opt_data_.D_alpha_n_norm(2) = (double)dL(2);

    std::cout << "alpha_n_norm: [" << std::to_string(opt_data_.D_alpha_n_norm(0)) << "," << std::to_string(opt_data_.D_alpha_n_norm(1)) << "," << std::to_string(opt_data_.D_alpha_n_norm(2)) << "]" << std::endl;

}

void PowerlinePositionsComputerNode::projMagPosSet() {

    point_t origin(0,0,0);
    vector_t tmp_norm(
        opt_data_.D_alpha_n_norm(0),
        opt_data_.D_alpha_n_norm(1),
        opt_data_.D_alpha_n_norm(2)
    );
    plane_t alpha = {
        .p = origin,
        .normal = tmp_norm
    };

    for (int i = 0; i < 4; i++) {

        point_t proj_pt = projectPointOnPlane((point_t)v_drone_to_mags_[i], alpha);
        opt_data_.D_Mjalpha_p[i](0) = (double)proj_pt(0); 
        opt_data_.D_Mjalpha_p[i](1) = (double)proj_pt(1); 
        opt_data_.D_Mjalpha_p[i](2) = (double)proj_pt(2); 

        std::cout << "D_M" << std::to_string(i) << "alpha_p: [" << std::to_string(proj_pt(0)) << "," << std::to_string(proj_pt(1)) << "," << std::to_string(proj_pt(2)) << "]" << std::endl;

    }

}

double PowerlinePositionsComputerNode::F(const std::vector<double> &x, std::vector<double> &grad, void *f_data) {

    std::cout << "Computing objective function" << std::endl;

    opt_data_t *data = (opt_data_t *)f_data;

    Eigen::Matrix<double, 12, 1> D_B;

    std::cout << "I: " << std::to_string(x[0]) << std::endl;

    double C = x[0] * data->C;

    std::cout << "C: " << std::to_string(C) << std::endl;

    vectord_t D_P1_p(x[1],x[2],x[3]);
    vectord_t D_P2_p(x[4],x[5],x[6]);

    std::cout << "D_P1_p: [" << std::to_string(x[1]) << "," << std::to_string(x[2]) << "," << std::to_string(x[3]) << "]" << std::endl;
    std::cout << "D_P2_p: [" << std::to_string(x[4]) << "," << std::to_string(x[5]) << "," << std::to_string(x[6]) << "]" << std::endl;

    std::cout << std::endl;

    for (int i = 0; i < 4; i++) {

        vectord_t D_a_1j = D_P1_p - data->D_Mjalpha_p[i];
        vectord_t cross_prod_1 = D_a_1j.cross(data->D_alpha_n_norm);
        double D_a_1j_norm = D_a_1j.norm();
        vectord_t D_B_1j = cross_prod_1 / pow(D_a_1j_norm, 3);

        std::cout << "D_a_1" << std::to_string(i) << ": [" << std::to_string(D_a_1j(0)) << "," << std::to_string(D_a_1j(1)) << "," << std::to_string(D_a_1j(2)) << "]" << std::endl;
        std::cout << "cross_prod_1" << std::to_string(i) << ": [" << std::to_string(cross_prod_1(0)) << "," << std::to_string(cross_prod_1(1)) << "," << std::to_string(cross_prod_1(2)) << "]" << std::endl;
        std::cout << "D_a_1" << std::to_string(i) << "_norm: " << std::to_string(D_a_1j_norm) << std::endl;
        std::cout << "D_B_1" << std::to_string(i) << ": [" << std::to_string(D_B_1j(0)) << "," << std::to_string(D_B_1j(1)) << "," << std::to_string(D_B_1j(2)) << "]" << std::endl;

        vectord_t D_a_2j = D_P2_p - data->D_Mjalpha_p[i];
        vectord_t cross_prod_2 = D_a_2j.cross(data->D_alpha_n_norm);
        double D_a_2j_norm = D_a_2j.norm();
        vectord_t D_B_2j = cross_prod_2 / pow(D_a_2j_norm, 3);

        std::cout << "D_a_2" << std::to_string(i) << ": [" << std::to_string(D_a_2j(0)) << "," << std::to_string(D_a_2j(1)) << "," << std::to_string(D_a_2j(2)) << "]" << std::endl;
        std::cout << "cross_prod_2" << std::to_string(i) << ": [" << std::to_string(cross_prod_2(0)) << "," << std::to_string(cross_prod_2(1)) << "," << std::to_string(cross_prod_2(2)) << "]" << std::endl;
        std::cout << "D_a_2" << std::to_string(i) << "_norm: " << std::to_string(D_a_2j_norm) << std::endl;
        std::cout << "D_B_2" << std::to_string(i) << ": [" << std::to_string(D_B_2j(0)) << "," << std::to_string(D_B_2j(1)) << "," << std::to_string(D_B_2j(2)) << "]" << std::endl;

        vectord_t D_B_j;
        if (data->inverted) {

            D_B_j = D_B_2j - D_B_1j;

        } else {

            D_B_j = D_B_1j - D_B_2j;

        }

        std::cout << "D_B_" << std::to_string(i) << ": [" << std::to_string(D_B_j(0)) << "," << std::to_string(D_B_j(1)) << "," << std::to_string(D_B_j(2)) << "]" << std::endl;

        D_B(i*3) = D_B_j(0);
        D_B(i*3+1) = D_B_j(1);
        D_B(i*3+2) = D_B_j(2);

        std::cout << std::endl;

    }

    D_B *= C * data->conversion_factor;

    std::cout << "D_B: [";
    for (int i = 0; i < 12; i++) {
        std::cout << std::to_string(D_B(i));
        if (i < 11) {
            std::cout << ",";
        }
    }
    std::cout << "]" << std::endl;

    std::cout << "D_V: [";
    for (int i = 0; i < 12; i++) {
        std::cout << std::to_string(data->D_V(i));
        if (i < 11) {
            std::cout << ",";
        }
    }
    std::cout << "]" << std::endl;

    Eigen::Matrix<double, 12, 1> error = D_B - data->D_V;
    std::cout << "error: [";
    for (int i = 0; i < 12; i++) {
        std::cout << std::to_string(error(i));
        if (i < 11) {
            std::cout << ",";
        }
    }
    std::cout << "]" << std::endl;

    double SSE = error.dot(error);

    std::cout << "SSE: " << std::to_string(SSE);

    if (grad.size() > 0) {

        std::vector<double> empty_vec;
        std::vector<double> x_cp = x;

        for (int i = 0; i < 7; i++) {

            double x_val = x_cp[i];
            double xmh;
            double xph;
            double h2;

            if (abs(x_val) < 1) {
                xmh = x_val-100*data->epsilon;
                xph = x_val+100*data->epsilon;
            } else {
                xmh = x_val * (1 - 100*data->epsilon);
                xph = x_val * (1 + 100*data->epsilon);
            }

            h2 = xph - xmh;

            x_cp[i] = xph;
            double fph = PowerlinePositionsComputerNode::F(x_cp, empty_vec, f_data);

            x_cp[i] = xmh;
            double fmh = PowerlinePositionsComputerNode::F(x_cp, empty_vec, f_data);

            x_cp[i] = x_val;

            grad[i] = (fph - fmh) / h2;

        }
    }

    std::cout << std::endl << std::endl;

    return SSE;

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
		v_drone_to_mags_[i](1) = mag_tf.transform.translation.x;
		v_drone_to_mags_[i](2) = mag_tf.transform.translation.x;

	}

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<PowerlinePositionsComputerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}