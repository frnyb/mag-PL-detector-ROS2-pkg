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

	this->declare_parameter<float>("kf_q", 0.5);
	this->get_parameter("kf_q", q_);

	this->declare_parameter<float>("kf_r_odom", 1-q_);
	this->get_parameter("kf_r_odom", r_odom_);

	this->declare_parameter<float>("kf_r_mag_vec", 1-q_);
	this->get_parameter("kf_r_mag_vec", r_mag_vec_);

	this->declare_parameter<float>("vector_parallel_dot_prod_thresh", 0.95);
	this->get_parameter("vector_parallel_dot_prod_thresh", parallel_dot_prod_thresh_);

	geometry_msgs::msg::TransformStamped mag0_tf;
	geometry_msgs::msg::TransformStamped mag1_tf;
	geometry_msgs::msg::TransformStamped mag2_tf;
	geometry_msgs::msg::TransformStamped mag3_tf;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	while(true) {

		try {

			mag0_tf = tf_buffer_->lookupTransform("drone", "mag0", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag0 transform, frame drone to mag0");

			break;

		} catch(tf2::TransformException & ex) {

			RCLCPP_INFO(this->get_logger(), "Could not get mag transform, trying again...");

		}
	}

	while(true) {

		try {

			mag1_tf = tf_buffer_->lookupTransform("drone", "mag1", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag1 transform, frame drone to mag1");
			
			break;

		} catch(tf2::TransformException & ex) {

			RCLCPP_INFO(this->get_logger(), "Could not get mag1 transform, trying again...");

		}
	}

	while(true) {

		try {

			mag2_tf = tf_buffer_->lookupTransform("drone", "mag2", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag2 transform, frame drone to mag3");

			break;

		} catch(tf2::TransformException & ex) {

			RCLCPP_INFO(this->get_logger(), "Could not get mag2 transform, trying again...");

		}
	}

	while(true) {

		try {

			mag3_tf = tf_buffer_->lookupTransform("drone", "mag3", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag3 transform, frame drone to mag3");

			break;

		} catch(tf2::TransformException & ex) {

			RCLCPP_INFO(this->get_logger(), "Could not get mag3 transform, trying again...");

		}
	}

	quat_t mag0_quat(
		mag0_tf.transform.rotation.w,
		mag0_tf.transform.rotation.x,
		mag0_tf.transform.rotation.y,
		mag0_tf.transform.rotation.z
	);
	quat_t mag1_quat(
		mag1_tf.transform.rotation.w,
		mag1_tf.transform.rotation.x,
		mag1_tf.transform.rotation.y,
		mag1_tf.transform.rotation.z
	);
	quat_t mag2_quat(
		mag2_tf.transform.rotation.w,
		mag2_tf.transform.rotation.x,
		mag2_tf.transform.rotation.y,
		mag2_tf.transform.rotation.z
	);
	quat_t mag3_quat(
		mag3_tf.transform.rotation.w,
		mag3_tf.transform.rotation.x,
		mag3_tf.transform.rotation.y,
		mag3_tf.transform.rotation.z
	);

	R_drone_to_mag0_ = quatToMat(mag0_quat);
	R_drone_to_mag1_ = quatToMat(mag1_quat);
	R_drone_to_mag2_ = quatToMat(mag2_quat);
	R_drone_to_mag3_ = quatToMat(mag3_quat);

	v_drone_to_mag0_(0) = mag0_tf.transform.translation.x;
	v_drone_to_mag0_(1) = mag0_tf.transform.translation.x;
	v_drone_to_mag0_(2) = mag0_tf.transform.translation.x;

	v_drone_to_mag1_(0) = mag1_tf.transform.translation.x;
	v_drone_to_mag1_(1) = mag1_tf.transform.translation.x;
	v_drone_to_mag1_(2) = mag1_tf.transform.translation.x;

	v_drone_to_mag2_(0) = mag2_tf.transform.translation.x;
	v_drone_to_mag2_(1) = mag2_tf.transform.translation.x;
	v_drone_to_mag2_(2) = mag2_tf.transform.translation.x;

	v_drone_to_mag3_(0) = mag3_tf.transform.translation.x;
	v_drone_to_mag3_(1) = mag3_tf.transform.translation.x;
	v_drone_to_mag3_(2) = mag3_tf.transform.translation.x;

	if (fixed_phase_) {

		mag0_amplitude_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
			"/mag0_amplitude_vector", 10, std::bind(&PowerlineDirectionComputerNode::mag0AmplitudeCallback, this, std::placeholders::_1));
		mag1_amplitude_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
			"/mag1_amplitude_vector", 10, std::bind(&PowerlineDirectionComputerNode::mag1AmplitudeCallback, this, std::placeholders::_1));
		mag2_amplitude_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
			"/mag2_amplitude_vector", 10, std::bind(&PowerlineDirectionComputerNode::mag2AmplitudeCallback, this, std::placeholders::_1));
		mag3_amplitude_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
			"/mag3_amplitude_vector", 10, std::bind(&PowerlineDirectionComputerNode::mag3AmplitudeCallback, this, std::placeholders::_1));

	} else {

		mag0_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
			"/mag0_phasor", 10, std::bind(&PowerlineDirectionComputerNode::mag0PhasorCallback, this, std::placeholders::_1));
		mag1_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
			"/mag1_phasor", 10, std::bind(&PowerlineDirectionComputerNode::mag1PhasorCallback, this, std::placeholders::_1));
		mag2_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
			"/mag2_phasor", 10, std::bind(&PowerlineDirectionComputerNode::mag2PhasorCallback, this, std::placeholders::_1));
		mag3_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
			"/mag3_phasor", 10, std::bind(&PowerlineDirectionComputerNode::mag3PhasorCallback, this, std::placeholders::_1));

	}

	pl_direction_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/powerline_direction", 10);

    //tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    mag_vector_timer_ = this->create_wall_timer(
      250ms, std::bind(&PowerlineDirectionComputerNode::updateFromMagVectors, this));

}

void PowerlineDirectionComputerNode::updateFromMagVectors() {

	vector_t vectors[4];

	ampl_vec_mutex_.lock(); {

		vectors[0] = norm_mag0_ampl_vec;
		vectors[1] = norm_mag1_ampl_vec;
		vectors[2] = norm_mag2_ampl_vec;
		vectors[3] = norm_mag3_ampl_vec;

	} ampl_vec_mutex_.unlock();

	vector_t cross_vec_sum(0,0,0);
	int cnt = 0;

	for (int i = 0; i < 3; i++) {

		for (int j = i+1; j < 4; j++) {

			if (abs(vectors[i].dot(vectors[j])) > parallel_dot_prod_thresh_) {

				continue;

			}

			vector_t cross_vec = vectors[i].cross(vectors[j]);
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

		return;

	}

	vector_t pl_dir = cross_vec_sum / cnt;
	pl_dir /= pl_dir.norm();

	quat_t q;

	vector_t unit_x(1,0,0);
	vector_t a = unit_x.cross(pl_dir);

	q(0) = unit_x.dot(pl_dir);
	q(1) = a(0);
	q(2) = a(1);
	q(3) = a(2);
	q /= q.norm();

	publishPowerlineDirection(q);

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

void PowerlineDirectionComputerNode::mag0AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag0_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag0_ampl_vec = ampl;
		norm_mag0_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionComputerNode::mag1AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag1_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag1_ampl_vec = ampl;
		norm_mag1_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionComputerNode::mag2AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag2_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag2_ampl_vec = ampl;
		norm_mag2_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionComputerNode::mag3AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag3_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag3_ampl_vec = ampl;
		norm_mag3_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionComputerNode::mag0PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag0_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag0_ampl_vec = ampl;
		norm_mag0_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionComputerNode::mag1PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag1_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag1_ampl_vec = ampl;
		norm_mag1_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionComputerNode::mag2PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag2_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag2_ampl_vec = ampl;
		norm_mag2_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionComputerNode::mag3PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag3_ * ampl;
	vector_t norm_ampl = ampl / ampl.norm();

	ampl_vec_mutex_.lock(); {

		mag3_ampl_vec = ampl;
		norm_mag3_ampl_vec = norm_ampl;

	} ampl_vec_mutex_.unlock();

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<PowerlineDirectionComputerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}