/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "pl_positions_estimator.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

PowerlinePositionsEstimatorNode::PowerlinePositionsEstimatorNode(const std::string & node_name, 
                            const std::string & node_namespace) : rclcpp::Node(node_name, node_namespace) {

	this->declare_parameter<float>("kf_q", 0.999);
	this->get_parameter("kf_q", kf_q_);

	this->declare_parameter<float>("kf_r", 1-kf_q_);
	this->get_parameter("kf_r", kf_r_);

	this->declare_parameter<float>("distance_discard_threshold", 3.);
	this->get_parameter("distance_discard_threshold", distance_discard_threshold_);

    RCLCPP_INFO(this->get_logger(), "Starting %s with parameters:", node_name);
	RCLCPP_INFO(this->get_logger(), "kf_q_: %f", kf_q_);
	RCLCPP_INFO(this->get_logger(), "kf_r_: %f", kf_r_);

	drone_quat_(0) = 1;
	drone_quat_(1) = 0;
	drone_quat_(2) = 0;
	drone_quat_(3) = 0;

	last_drone_quat_(0) = 1;
	last_drone_quat_(1) = 0;
	last_drone_quat_(2) = 0;
	last_drone_quat_(3) = 0;

	drone_pos_(0) = 0;
	drone_pos_(1) = 0;
	drone_pos_(2) = 0;

	last_drone_pos_(0) = 0;
	last_drone_pos_(1) = 0;
	last_drone_pos_(2) = 0;

	pl0_position_est_(0) = 0;
	pl0_position_est_(1) = 0;
	pl0_position_est_(2) = 0;

	pl1_position_est_(0) = 0;
	pl1_position_est_(1) = 0;
	pl1_position_est_(2) = 0;

	pl_current_est_ = 0;

	pl_dir_(0) = 1;
	pl_dir_(1) = 0;
	pl_dir_(2) = 0;
	pl_dir_(3) = 0;

	point_t origin(0,0,0);
	vector_t x(1,0,0);
	projection_plane_ = {
		.p = origin,
		.normal = x
	};

    has_updated_p0_ = false;
    has_updated_p1_ = false;

	for (int i = 0; i < 3; i++) {

		pl0_kf_[i].state_est = 0;
		pl0_kf_[i].var_est = 1;

		pl1_kf_[i].state_est = 0;
		pl1_kf_[i].var_est = 1;
	}

	pl_poses_result_raw_sub_ = this->create_subscription<mag_pl_detector::msg::PowerlinePosesComputationResult>(
		"/pl_positions_computer/pl_poses_raw", 10, std::bind(&PowerlinePositionsEstimatorNode::powerlinePosesRawCallback, this, std::placeholders::_1));

    pl_poses_result_est_pub_ = this->create_publisher<mag_pl_detector::msg::PowerlinePosesComputationResult>(
        "pl_poses_est", 10
    );

    pl0_pose_est_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pl0_pose_est", 10
    );

    pl1_pose_est_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pl1_pose_est", 10
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odometry_timer_ = this->create_wall_timer(
      10ms, std::bind(&PowerlinePositionsEstimatorNode::odometryCallback, this));

}

void PowerlinePositionsEstimatorNode::odometryCallback() {

	if (fetchDronePose()) {

		kfPredict();

		publishPowerlinePosesEst();

	}
}

void PowerlinePositionsEstimatorNode::powerlinePosesRawCallback(mag_pl_detector::msg::PowerlinePosesComputationResult::SharedPtr msg) {

	if (msg->poses.size() > 0) {

		pl_dir_mutex_.lock(); {

			geometry_msgs::msg::Pose pose = msg->poses[0];

			quat_t dir(
				pose.orientation.w,
				pose.orientation.x,
				pose.orientation.y,
				pose.orientation.z
			);

			pl_dir_ = dir;

			rotation_matrix_t pl_R = quatToMat(pl_dir_);
			vector_t unit_x(1,0,0);
			vector_t pl_unit_x = pl_R * unit_x;

			point_t origin(0,0,0);

			projection_plane_ = {
				.p = origin,
				.normal = pl_unit_x
			};

		} pl_dir_mutex_.unlock();
	}

	current_mutex_.lock(); {

		pl_current_est_ = msg->current;

	} current_mutex_.unlock();

	kfUpdate(msg->poses);

}

bool PowerlinePositionsEstimatorNode::fetchDronePose() {

    RCLCPP_DEBUG(this->get_logger(), "Fetching odometry transform");

    geometry_msgs::msg::TransformStamped tf;

	std::vector<std::string> transform_names = tf_buffer_->getAllFrameNames();

	try {

		tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

	} catch(tf2::TransformException & ex) {

		RCLCPP_DEBUG(this->get_logger(), "Could not get odometry transform, frame world to drone");
		return false;

	}

    quat_t quat(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z
    );

	last_drone_quat_ = drone_quat_;
	drone_quat_ = quat;

	vector_t vec(
		tf.transform.translation.x,
		tf.transform.translation.y,
		tf.transform.translation.z
	);

	last_drone_pos_ = drone_pos_;
	drone_pos_ = vec;

	return true;

}

void PowerlinePositionsEstimatorNode::kfPredict() {

    quat_t inv_last_drone_quat = quatInv(last_drone_quat_);
    quat_t delta_drone_quat = quatMultiply(drone_quat_, inv_last_drone_quat);
	rotation_matrix_t delta_drone_R = quatToMat(delta_drone_quat);

	vector_t delta_drone_p = drone_pos_ - last_drone_pos_;

	plane_t proj_plane;

	pl_dir_mutex_.lock(); {

		proj_plane = projection_plane_;

	} pl_dir_mutex_.unlock();

	vector_t pl0_pos;
	vector_t pl1_pos;

    kf_mutex_.lock(); {

		pl0_pos(0) = pl0_kf_[0].state_est;
		pl0_pos(1) = pl0_kf_[1].state_est;
		pl0_pos(2) = pl0_kf_[2].state_est;

		pl1_pos(0) = pl1_kf_[0].state_est;
		pl1_pos(1) = pl1_kf_[1].state_est;
		pl1_pos(2) = pl1_kf_[2].state_est;

		pl0_pos = delta_drone_R * pl0_pos + delta_drone_p;
		pl1_pos = delta_drone_R * pl1_pos + delta_drone_p;

		pl0_pos = (vector_t)projectPointOnPlane((point_t)pl0_pos, proj_plane);
		pl1_pos = (vector_t)projectPointOnPlane((point_t)pl1_pos, proj_plane);

		for (int i = 0; i < 3; i++) {

			pl0_kf_[i].state_est = pl0_pos(i);
			pl0_kf_[i].var_est += kf_q_;

			pl1_kf_[i].state_est = pl1_pos(i);
			pl1_kf_[i].var_est += kf_q_;

		}

    } kf_mutex_.unlock();

    pl_positions_mutex_.lock(); {

        pl0_position_est_ = pl0_pos;
        pl1_position_est_ = pl1_pos;

    } pl_positions_mutex_.unlock();
}

void PowerlinePositionsEstimatorNode::kfUpdate(std::vector<geometry_msgs::msg::Pose> poses) {

	using namespace std;

	bool *vec0_update_ptr;
	bool *vec1_update_ptr;

	vector_t vec0;
	vector_t vec1;

	kf_est_t *recv0_kf_ptr;
	kf_est_t *recv1_kf_ptr;

	vector_t *recv0_vec_ptr;
	vector_t *recv1_vec_ptr;

	vector_t vec0_estimate;
	vector_t vec1_estimate;

	pl_positions_mutex_.lock(); {

		vec0_estimate = pl0_position_est_;
		vec1_estimate = pl1_position_est_;

	} pl_positions_mutex_.unlock();

	kf_mutex_.lock(); {

		if (poses.size() > 0) {

			vec0(0) = poses[0].position.x;
			vec0(1) = poses[0].position.y;
			vec0(2) = poses[0].position.z;

			if ((vec0 - vec0_estimate).norm() > (vec0 - vec1_estimate).norm()) {

				recv0_kf_ptr = pl1_kf_;
				recv1_kf_ptr = pl0_kf_;

				vec0_update_ptr = &has_updated_p1_;
				vec1_update_ptr = &has_updated_p0_;

				recv0_vec_ptr = &vec1_estimate;
				recv1_vec_ptr = &vec0_estimate;

			} else {

				recv0_kf_ptr = pl0_kf_;
				recv1_kf_ptr = pl1_kf_;

				recv0_vec_ptr = &vec0_estimate;
				recv1_vec_ptr = &vec1_estimate;

				vec1_update_ptr = &has_updated_p1_;
				vec0_update_ptr = &has_updated_p0_;

			}

			if (poses.size() > 1) {

				vec1(0) = poses[1].position.x;
				vec1(1) = poses[1].position.y;
				vec1(2) = poses[1].position.z;

				if ((vec1 - *recv0_vec_ptr).norm() < (vec0 - *recv0_vec_ptr).norm()) {

					kf_est_t *tmp_kf_ptr = recv0_kf_ptr;
					recv0_kf_ptr = recv1_kf_ptr;
					recv1_kf_ptr = tmp_kf_ptr;

					vector_t *tmp_vec_ptr = recv0_vec_ptr;
					recv1_vec_ptr = recv0_vec_ptr;
					recv0_vec_ptr = tmp_vec_ptr;

	 				bool *tmp_bool_ptr = vec0_update_ptr;
					vec1_update_ptr = vec0_update_ptr;
					vec0_update_ptr = tmp_bool_ptr;

				}

				if (!has_updated_p0_ || !has_updated_p1_ || (vec1 - *recv1_vec_ptr).norm() < distance_discard_threshold_) {

					update(recv1_kf_ptr, vec1);
					*vec1_update_ptr = true;

				}
			}

			if (!has_updated_p0_ || !has_updated_p1_ || (vec0 - vec0_estimate).norm() < distance_discard_threshold_) {

				update(recv0_kf_ptr, vec0);
				*vec0_update_ptr = true;

			}
		}

		vec0_estimate(0) = pl0_kf_[0].state_est;
		vec0_estimate(1) = pl0_kf_[1].state_est;
		vec0_estimate(2) = pl0_kf_[2].state_est;

		vec1_estimate(0) = pl1_kf_[0].state_est;
		vec1_estimate(1) = pl1_kf_[1].state_est;
		vec1_estimate(2) = pl1_kf_[2].state_est;



	} kf_mutex_.unlock();

	pl_positions_mutex_.lock(); {

		pl0_position_est_ = vec0_estimate;

		pl1_position_est_ = vec1_estimate;

	} pl_positions_mutex_.unlock();

}

void PowerlinePositionsEstimatorNode::update(kf_est_t *kf, vector_t new_vec) {

	for (int i = 0; i < 3; i++) {

		float y_bar = new_vec(i) - kf[i].state_est;
		float s = kf[i].var_est + kf_r_;

		float k = kf[i].var_est / s;

		kf[i].state_est += k*y_bar;
		kf[i].var_est *= 1-k;
		
	}
}

void PowerlinePositionsEstimatorNode::publishPowerlinePosesEst() {

    mag_pl_detector::msg::PowerlinePosesComputationResult result_msg;
    result_msg.header.frame_id = "drone";
    result_msg.header.stamp = this->get_clock()->now();

	current_mutex_.lock(); {

		result_msg.current = pl_current_est_;

	} current_mutex_.unlock();

	quat_t pl_dir;

	pl_dir_mutex_.lock(); {

		pl_dir = pl_dir_;

	} pl_dir_mutex_.unlock();

	std::vector<geometry_msgs::msg::Pose> poses;

	pl_positions_mutex_.lock(); {

		geometry_msgs::msg::Pose pose;

		pose.orientation.w = pl_dir(0);
		pose.orientation.x = pl_dir(1);
		pose.orientation.y = pl_dir(2);
		pose.orientation.z = pl_dir(3);

		pose.position.x = pl0_position_est_(0);
		pose.position.y = pl0_position_est_(1);
		pose.position.z = pl0_position_est_(2);

		poses.push_back(pose);

		pose.position.x = pl1_position_est_(0);
		pose.position.y = pl1_position_est_(1);
		pose.position.z = pl1_position_est_(2);

		poses.push_back(pose);

	} pl_positions_mutex_.unlock();

    result_msg.poses = poses;

    pl_poses_result_est_pub_->publish(result_msg);

	geometry_msgs::msg::PoseStamped msg0;
	msg0.header.frame_id = "drone";
	msg0.header.stamp = result_msg.header.stamp;

	msg0.pose = poses[0];

	pl0_pose_est_pub_->publish(msg0);

	geometry_msgs::msg::PoseStamped msg1;
	msg1.header.frame_id = "drone";
	msg1.header.stamp = result_msg.header.stamp;

	msg1.pose = poses[1];

	pl1_pose_est_pub_->publish(msg1);

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<PowerlinePositionsEstimatorNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}