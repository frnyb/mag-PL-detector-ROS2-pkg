/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "pl_dir_computer.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

PowerlineDirectionComputerNode::PowerlineDirectionComputerNode(const std::string & node_name, 
                            const std::string & node_namespace) : rclcpp::Node(node_name, node_namespace) {


	this->declare_parameter<bool>("fixed_phase", true);
	this->get_parameter("fixed_phase", fixed_phase_);
	
	this->declare_parameter<int>("direction_computation_method", 1);
	int dir_method;
	this->get_parameter("direction_computation_method", dir_method);

	this->declare_parameter<int>("direction_computation_buffer_size", 5);
	int buffer_size;
	this->get_parameter("direction_computation_buffer_size", buffer_size);

	this->declare_parameter<float>("vector_parallel_dot_prod_thresh", 0.95);
	this->get_parameter("vector_parallel_dot_prod_thresh", parallel_dot_prod_thresh_);

    RCLCPP_INFO(this->get_logger(), "Starting %s with parameters: ", node_name);
	RCLCPP_INFO(this->get_logger(), "fixed_phase: %s", std::to_string(fixed_phase_));
	RCLCPP_INFO(this->get_logger(), "vector_parallel_dot_prod_thresh: %f", parallel_dot_prod_thresh_);
	RCLCPP_INFO(this->get_logger(), "direction_computation_method: %d", dir_method);
	RCLCPP_INFO(this->get_logger(), "direction_computation_buffer_size: %d", buffer_size);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	fetchStaticTransforms();

    odometry_timer_ = this->create_wall_timer(
      10ms, std::bind(&PowerlineDirectionComputerNode::odometryCallback, this));

	mag_phasors_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasors3D>(
		"/sine_reconstructor/mag_phasors", 10, std::bind(&PowerlineDirectionComputerNode::magPhasorsCallback, this, std::placeholders::_1));

	pl_direction_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("powerline_direction_raw", 10);

	const std::string mag_norm_amp_topics[4] = {
		"mag0_norm_mag_ampl_vector",
		"mag1_norm_mag_ampl_vector",
		"mag2_norm_mag_ampl_vector",
		"mag3_norm_mag_ampl_vector"
	};

	for (int i = 0; i < 4; i++) {

		mag_norm_ampl_pubs_[i] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(mag_norm_amp_topics[i], 10);

	}

}

void PowerlineDirectionComputerNode::odometryCallback() {

    RCLCPP_DEBUG(this->get_logger(), "Fetching odometry transform");

    geometry_msgs::msg::TransformStamped tf;

	std::vector<std::string> transform_names = tf_buffer_->getAllFrameNames();

	try {

		tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

	} catch(tf2::TransformException & ex) {

		RCLCPP_DEBUG(this->get_logger(), "Could not get odometry transform, frame world to drone");

	}

    quat_t quat(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z
    );

	odometry_mutex_.lock(); {

		drone_quat_ = quat;

	} odometry_mutex_.unlock();

}

void PowerlineDirectionComputerNode::computePowerlineDirection() {

	int direction_computation_method;
	this->get_parameter("direction_computation_method", direction_computation_method);

	switch (direction_computation_method) {

	default:
	case 0:
		pl_dir_quat_ = crossVectorMeanDirectionComputation();
		break;

	case 1:
		pl_dir_quat_ = projectionVectorMeanDirectionComputation();
		break;

	case 2:
		pl_dir_quat_ = projectionLineFitDirectionComputation();
		break;

	case 3:
		pl_dir_quat_ = projectionBufferLineFitDirectionComputation();
		break;

	}
}

quat_t PowerlineDirectionComputerNode::crossVectorMeanDirectionComputation() {

	vector_t cross_vec_sum(0,0,0);
	int cnt = 0;

	for (int i = 0; i < 3; i++) {

		for (int j = i+1; j < 4; j++) {

			if (abs(norm_mag_ampl_vecs_[i].dot(norm_mag_ampl_vecs_[j])) > parallel_dot_prod_thresh_) {

				continue;

			}

			vector_t cross_vec = norm_mag_ampl_vecs_[i].cross(norm_mag_ampl_vecs_[j]);
			cross_vec /= cross_vec.norm();

			if (cnt > 0) {

				if (cross_vec_sum.dot(cross_vec) < 0) {

					cross_vec = -cross_vec;

				}
			}

			cross_vec_sum += cross_vec;
			cnt++;

		}
	}

	if (cnt == 0) {

		return pl_dir_quat_;

	}

	vector_t pl_dir = cross_vec_sum / cnt;
	pl_dir /= pl_dir.norm();

	quat_t q;

	vector_t unit_x(1,0,0);
	vector_t a = pl_dir.cross(unit_x);

	q(0) = unit_x.dot(pl_dir);
	q(1) = a(0);
	q(2) = a(1);
	q(3) = a(2);
	q /= q.norm();

	return q;

}

quat_t PowerlineDirectionComputerNode::projectionVectorMeanDirectionComputation() {

	quat_t drone_quat;

	odometry_mutex_.unlock(); {

		drone_quat = drone_quat_;

	} odometry_mutex_.unlock();

	rotation_matrix_t W_D_R = quatToMat(drone_quat);
	
	vector_t W_v[4];

	vector_2D_t avg_vec(0,0);

	for (int i = 0; i < 4; i++) {

		W_v[i] = W_D_R * norm_mag_ampl_vecs_[i];

		if (i > 0 && W_v[0].dot(W_v[i]) < 0) {

			W_v[i] = -W_v[i];

		}

		vector_2D_t v2d(W_v[i](0), W_v[i](1));

		avg_vec += v2d;

	}

	avg_vec /= 4;
	//avg_vec /= avg_vec.norm();

	vector_t normal_vec(avg_vec(1), -avg_vec(0), 0);
	vector_t pl_dir = W_D_R.transpose() * normal_vec;
	pl_dir /= pl_dir.norm();

	quat_t q;

	vector_t unit_x(1,0,0);
	vector_t a = pl_dir.cross(unit_x);

	q(0) = unit_x.dot(pl_dir);
	q(1) = a(0);
	q(2) = a(1);
	q(3) = a(2);
	q /= q.norm();

	return q;

}

quat_t PowerlineDirectionComputerNode::projectionLineFitDirectionComputation() {

	quat_t drone_quat;

	odometry_mutex_.unlock(); {

		drone_quat = drone_quat_;

	} odometry_mutex_.unlock();

	rotation_matrix_t W_D_R = quatToMat(drone_quat);
	
	vector_t W_v[4];

	vector_2D_t lls_points[9];
	vector_2D_t zero_vec(0,0);
	lls_points[0] = zero_vec;

	for (int i = 0; i < 4; i++) {

		W_v[i] = W_D_R * norm_mag_ampl_vecs_[i];

		vector_2D_t v2d(W_v[i](0), W_v[i](1));
		vector_2D_t mv2d(-W_v[i](0), -W_v[i](1));

		lls_points[i+1] = v2d;
		lls_points[i+5] = mv2d;

	}

	Eigen::MatrixXf A(9, 2);
	Eigen::VectorXf b(9);

	for (int i = 0; i < 9; i++) {

		A(i,0) = lls_points[i](0);
		A(i,1) = 1;

		b(i) = lls_points[i](1);

	}

	Eigen::Vector2f x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	vector_2D_t point0(
		0,
		x(1)
	);

	vector_2D_t point1(
		1,
		x(0) + x(1)
	);

	vector_2D_t dir_vec = point1 - point0;

	vector_t norm_vec(dir_vec(1), -dir_vec(0), 0);

	vector_t pl_dir = W_D_R.transpose() * norm_vec;
	pl_dir /= pl_dir.norm();

	quat_t q;

	vector_t unit_x(1,0,0);
	vector_t a = pl_dir.cross(unit_x);

	q(0) = unit_x.dot(pl_dir);
	q(1) = a(0);
	q(2) = a(1);
	q(3) = a(2);
	q /= q.norm();

	return q;

}

quat_t PowerlineDirectionComputerNode::projectionBufferLineFitDirectionComputation() {

	quat_t drone_quat;

	odometry_mutex_.unlock(); {

		drone_quat = drone_quat_;

	} odometry_mutex_.unlock();

	rotation_matrix_t W_D_R = quatToMat(drone_quat);
	
	vector_t W_v[4];

	std::vector<vector_2D_t> points_2D;

	for (int i = 0; i < 4; i++) {

		W_v[i] = W_D_R * norm_mag_ampl_vecs_[i];

		vector_2D_t v2d(W_v[i](0), W_v[i](1));
		vector_2D_t mv2d(-W_v[i](0), -W_v[i](1));

		points_2D.push_back(v2d);
		points_2D.push_back(mv2d);

	}

	points_buffer.Push(points_2D);

	int buffer_size;
	this->get_parameter("direction_computation_buffer_size", buffer_size);

	if (points_buffer.Size() < buffer_size) {

		return pl_dir_quat_;

	}

	std::vector<vector_2D_t> all_points;

	for (int i = 0; i < buffer_size; i++) {

		std::vector<vector_2D_t> points = points_buffer[i];

		all_points.insert( all_points.end(), points.begin(), points.end() );

	}

	vector_2D_t zero_vec(0,0);
	all_points.push_back(zero_vec);

	Eigen::MatrixXf A(all_points.size(), 2);
	Eigen::VectorXf b(all_points.size());


	for (int i = 0; i < all_points.size(); i++) {

		A(i,0) = all_points[i](0);
		A(i,1) = 1;

		b(i) = all_points[i](1);

	}

	Eigen::Vector2f x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	vector_2D_t point0(
		0,
		x(1)
	);

	vector_2D_t point1(
		1,
		x(0) + x(1)
	);

	vector_2D_t dir_vec = point1 - point0;

	vector_t norm_vec(dir_vec(1), -dir_vec(0), 0);

	vector_t pl_dir = W_D_R.transpose() * norm_vec;
	pl_dir /= pl_dir.norm();

	quat_t q;

	vector_t unit_x(1,0,0);
	vector_t a = pl_dir.cross(unit_x);

	q(0) = unit_x.dot(pl_dir);
	q(1) = a(0);
	q(2) = a(1);
	q(3) = a(2);
	q /= q.norm();

	return q;

}

void PowerlineDirectionComputerNode::publishPowerlineDirection(quat_t q) {

	geometry_msgs::msg::PoseStamped msg;

	msg.header.frame_id = "drone";
	msg.header.stamp = this->get_clock()->now();

	msg.pose.orientation.w = q(0);
	msg.pose.orientation.x = q(1);
	msg.pose.orientation.y = q(2);
	msg.pose.orientation.z = q(3);

	msg.pose.position.x = 0;
	msg.pose.position.y = 0;
	msg.pose.position.z = 0;

	pl_direction_pub_->publish(msg);

}

void PowerlineDirectionComputerNode::publishNormMagAmpVecs(vector_t vecs[4]) {

	rclcpp::Time stamp = this->get_clock()->now();

	for (int i = 0; i < 4; i++) {

		geometry_msgs::msg::Vector3Stamped msg;

		msg.header.frame_id = mag_frame_names_[i];
		msg.header.stamp = stamp;

		msg.vector.x = norm_mag_ampl_vecs_[i](0);
		msg.vector.y = norm_mag_ampl_vecs_[i](1);
		msg.vector.z = norm_mag_ampl_vecs_[i](2);

		mag_norm_ampl_pubs_[i]->publish(msg);

	}
}

void PowerlineDirectionComputerNode::getAmplitudes(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr phasors_msg) {

	for (int i = 0; i < 4; i++) {

		mag_pl_detector::msg::MagneticPhasor3D phasor_msg = phasors_msg->phasors[i];

		vector_t ampl(
			phasor_msg.amplitudes.x,
			phasor_msg.amplitudes.y,
			phasor_msg.amplitudes.z
		);

		vector_t norm_ampl(
			phasor_msg.normalized_amplitudes.x,
			phasor_msg.normalized_amplitudes.y,
			phasor_msg.normalized_amplitudes.z
		);

		mag_ampl_vecs_[i] = ampl;
		norm_mag_ampl_vecs_[i] = norm_ampl;

	}

	correctVectors();
}

void PowerlineDirectionComputerNode::magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg) {

	getAmplitudes(msg);
	computePowerlineDirection();

	publishPowerlineDirection(pl_dir_quat_);
	publishNormMagAmpVecs(norm_mag_ampl_vecs_);

}

void PowerlineDirectionComputerNode::correctVectors() {

	vector_t v_proj_mags[4];
	vector_t P_D_Mja_p[4];

    rotation_matrix_t pl_R = quatToMat(pl_dir_quat_);
	rotation_matrix_t D_P_R_T = pl_R.transpose();
    vector_t unit_x(1,0,0);
    vector_t pl_unit_x = pl_R * unit_x;
    //pl_unit_x_(0) = 0.866;
    //pl_unit_x_(1) = 0.5;
    //pl_unit_x_(2) = 0;

    point_t origin(0,0,0);

    plane_t projection_plane = {
        .p = origin,
        .normal = pl_unit_x
    };

    for (int i = 0; i < 4; i++) {

        vector_t D_Malpha_pi = (vector_t)projectPointOnPlane((point_t)v_drone_to_mags_[i], projection_plane);
        v_proj_mags[i] = D_Malpha_pi;

    }

    for (int i = 0; i < 4; i++) {

        P_D_Mja_p[i] = D_P_R_T * v_proj_mags[i];

    }

    int misaligned_votes[] = {0,0,0,0};

    for (int i = 0; i < 4; i++) {

        vector_t D_vj = mag_ampl_vecs_[i];
		vector_t norm_D_vj = norm_mag_ampl_vecs_[i];

        if (i == 2) {
            D_vj = -D_vj;
			norm_D_vj = -norm_D_vj;
        }

        mag_ampl_vecs_[i] = D_vj;
        norm_mag_ampl_vecs_[i] = norm_D_vj;

        for (int j = 0; j < i; j++) {

            // cout << to_string(D_vj_norm_vecs[i].dot(D_vj_norm_vecs[j])) << "\n";

            if (norm_mag_ampl_vecs_[i].dot(norm_mag_ampl_vecs_[j]) < 0.5) {

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

            vector_t dist_vec = P_D_Mja_p[misaligned_index] - P_D_Mja_p[i];
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

                vector_t vec_rotated = R * norm_mag_ampl_vecs_[misaligned_index];

                double dot_prod = vec_rotated.dot(norm_mag_ampl_vecs_[compare_index]);
                if (dot_prod > best_dot_prod) {
                    best_dot_prod = dot_prod;
                    best_R = R;
                }
            }
        }

        mag_ampl_vecs_[misaligned_index] = best_R * mag_ampl_vecs_[misaligned_index];
    }


}

void PowerlineDirectionComputerNode::fetchStaticTransforms() {

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

	auto node = std::make_shared<PowerlineDirectionComputerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}
