/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "pl_dir_estimator.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

PowerlineDirectionEstimatorNode::PowerlineDirectionEstimatorNode(const std::string & node_name, 
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

	while(true) {

		try {
			mag0_tf = tf_buffer_->lookupTransform("drone", "mag0", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag0 transform, frame drone to mag0");

			mag1_tf = tf_buffer_->lookupTransform("drone", "mag1", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag1 transform, frame drone to mag0");

			mag2_tf = tf_buffer_->lookupTransform("drone", "mag2", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag2 transform, frame drone to mag0");

			mag3_tf = tf_buffer_->lookupTransform("drone", "mag3", tf2::TimePointZero);
			RCLCPP_INFO(this->get_logger(), "Found mag3 transform, frame drone to mag0");

		} catch(tf2::TransformException & ex) {

			RCLCPP_INFO(this->get_logger(), "Could not get mag transform, trying again...");

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
			"/mag0_amplitude_vector", 10, std::bind(&PowerlineDirectionEstimatorNode::mag0AmplitudeCallback, this, std::placeholders::_1));
		//mag1_amplitude_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
		//	"/mag1_amplitude_vector", 10, std::bind(&PowerlineDirectionEstimatorNode::mag1AmplitudeCallback, this, std::placeholders::_1));
		//mag2_amplitude_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
		//	"/mag2_amplitude_vector", 10, std::bind(&PowerlineDirectionEstimatorNode::mag2AmplitudeCallback, this, std::placeholders::_1));
		//mag3_amplitude_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
		//	"/mag3_amplitude_vector", 10, std::bind(&PowerlineDirectionEstimatorNode::mag3AmplitudeCallback, this, std::placeholders::_1));

	} else {

		//mag0_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
		//	"/mag0_phasor", 10, std::bind(&PowerlineDirectionEstimatorNode::mag0PhasorCallback, this, std::placeholders::_1));
		//mag1_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
		//	"/mag1_phasor", 10, std::bind(&PowerlineDirectionEstimatorNode::mag1PhasorCallback, this, std::placeholders::_1));
		//mag2_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
		//	"/mag2_phasor", 10, std::bind(&PowerlineDirectionEstimatorNode::mag2PhasorCallback, this, std::placeholders::_1));
		//mag3_phasor_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasor>(
		//	"/mag3_phasor", 10, std::bind(&PowerlineDirectionEstimatorNode::mag3PhasorCallback, this, std::placeholders::_1));

	}

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odometry_timer_ = this->create_wall_timer(
      25ms, std::bind(&PowerlineDirectionEstimatorNode::odometryCallback, this));

    mag_vector_timer_ = this->create_wall_timer(
      50ms, std::bind(&PowerlineDirectionEstimatorNode::updateFromMagVectors, this));

}

void PowerlineDirectionEstimatorNode::odometryCallback() {

    RCLCPP_DEBUG(this->get_logger(), "Fetching odometry transform");

    geometry_msgs::msg::TransformStamped tf;

    try {

        tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

    } catch(tf2::TransformException & ex) {

        RCLCPP_FATAL(this->get_logger(), "Could not get odometry transform, frame world to drone");
        return;

    }

    point_t position(
        tf.transform.translation.x,
        tf.transform.translation.y, 
        tf.transform.translation.z
    );

    quat_t quat(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z
    );

	odometry_mutex_.lock(); {

		last_drone_quat_ = drone_quat_;
		drone_quat_ = quat;

	}

    predict();

	updateFromOdometry();

	direction_mutex_.lock(); {

		publishPowerlineDirection(pl_direction_);

	} direction_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::predict() {

    quat_t inv_last_drone_quat = quatInv(last_drone_quat_);
    quat_t inv_drone_quat = quatInv(drone_quat_);
    quat_t delta_drone_quat = quatMultiply(drone_quat_, inv_last_drone_quat);

    orientation_t delta_drone_eul = quatToEul(delta_drone_quat);

    float delta_yaw = delta_drone_eul(2);
    float delta_pitch = delta_drone_eul(1);
    float pl_dir_yaw;
	float pl_dir_pitch;

    kf_mutex_.lock(); {

        pl_yaw_est_.state_est = pl_yaw_est_.state_est + delta_yaw;
        pl_yaw_est_.var_est += q_;

        pl_pitch_est_.state_est = pl_pitch_est_.state_est + delta_pitch;
        pl_pitch_est_.var_est += q_;

        pl_yaw_est_.state_est = backmapAngle(pl_yaw_est_.state_est);
        pl_dir_yaw = pl_yaw_est_.state_est;

        pl_pitch_est_.state_est = backmapAngle(pl_pitch_est_.state_est);
        pl_dir_pitch = pl_pitch_est_.state_est;

    } kf_mutex_.unlock();

    orientation_t pl_dir_eul(
        0,
        pl_dir_pitch,
        pl_dir_yaw
    );

    quat_t pl_dir = eulToQuat(pl_dir_eul);

    direction_mutex_.lock(); {

        pl_direction_ = pl_dir;

    } direction_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::updateFromOdometry() {

	orientation_t odom_eul;

	odometry_mutex_.lock(); {

		odom_eul = quatToEul(drone_quat_);

	} odometry_mutex_.unlock();

	float pitch = -odom_eul(1);

	kf_mutex_.lock(); {

		pl_pitch_est_ = update(r_odom_, pl_pitch_est_, pitch);
		pitch = pl_pitch_est_.state_est;

	} kf_mutex_.unlock();

	direction_mutex_.lock(); {

		orientation_t dir_eul = quatToEul(pl_direction_);
		dir_eul(1) = pitch;
		pl_direction_ = eulToQuat(dir_eul);

	} direction_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::updateFromMagVectors() {

	vector_t vectors[4];

	ampl_vec_mutex_.lock(); {

		vectors[0] = mag0_ampl_vec;
		vectors[1] = mag1_ampl_vec;
		vectors[2] = mag2_ampl_vec;
		vectors[3] = mag3_ampl_vec;

	} ampl_vec_mutex_.unlock();

	vector_t cross_vec_sum(0,0,0);
	int cnt = 0;

	for (int i = 0; i < 3; i++) {

		for (int j = i+1; j < 4; j++) {

			if (abs(vectors[i].dot(vectors[i])) > parallel_dot_prod_thresh_) {

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

	orientation_t eul = quatToEul(q);

	kf_mutex_.lock(); {

		pl_pitch_est_ = update(r_mag_vec_, pl_pitch_est_, eul(1));
		pl_yaw_est_ = update(r_mag_vec_, pl_yaw_est_, eul(2));

		eul(0) = 0;
		eul(1) = pl_pitch_est_.state_est;
		eul(2) = pl_yaw_est_.state_est;

	} kf_mutex_.unlock();

	q = eulToQuat(eul);

	direction_mutex_.lock(); {

		pl_direction_ = q;

	} direction_mutex_.lock();

	publishMagPowerlineDirection(q);

}

kf_est_t PowerlineDirectionEstimatorNode::update(float r, kf_est_t old_est, float new_x) {

		float new_angle = mapAngle(old_est.state_est, new_x);

		float y_bar = new_angle - old_est.state_est;
		float s = old_est.var_est + r;

		float k = old_est.var_est / s;

		old_est.state_est += k*y_bar;
		old_est.var_est *= 1-k;

		old_est.state_est = backmapAngle(old_est.state_est);

		return old_est;

}

float PowerlineDirectionEstimatorNode::backmapAngle(float angle) {

    if (angle > M_PI) {
        return angle-2*M_PI;
    } else if (angle < -M_PI) {
        return angle+2*M_PI;
    } else {
        return angle;
    }

}

float PowerlineDirectionEstimatorNode::mapAngle(float curr_angle, float new_angle) {

    float angle_candidates[4];
    angle_candidates[0] = new_angle;

    if (new_angle > 0) {

        angle_candidates[1] = new_angle - M_PI;
        angle_candidates[2] = new_angle - 2*M_PI;
        angle_candidates[3] = new_angle + M_PI;

    } else {

        angle_candidates[1] = new_angle + M_PI;
        angle_candidates[2] = new_angle + 2*M_PI;
        angle_candidates[3] = new_angle - M_PI;

    }

    float best_angle = angle_candidates[0];
    float best_angle_diff = abs(angle_candidates[0]-curr_angle);

    for (int i = 0; i < 4; i++) {

        float diff = abs(angle_candidates[i]-curr_angle);
        if (diff < best_angle_diff) {
            best_angle_diff = diff;
            best_angle = angle_candidates[i];
        }

    }

    return best_angle;

}

void PowerlineDirectionEstimatorNode::publishPowerlineDirection(quat_t q) {

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

void PowerlineDirectionEstimatorNode::publishMagPowerlineDirection(quat_t q) {

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

	mag_pl_direction_pub_->publish(msg);

}

void PowerlineDirectionEstimatorNode::mag0AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag0_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag0_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::mag1AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag1_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag1_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::mag2AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag2_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag2_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::mag3AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

	vector_t ampl(
		msg->vector.x,
		msg->vector.y,
		msg->vector.z
	);

	ampl = R_drone_to_mag3_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag3_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::mag0PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag0_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag0_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::mag1PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag1_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag1_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::mag2PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag2_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag2_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::mag3PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg) {

	vector_t ampl(
		msg->amplitudes.x,
		msg->amplitudes.y,
		msg->amplitudes.z
	);

	ampl = R_drone_to_mag3_ * ampl;

	ampl_vec_mutex_.lock(); {

		mag3_ampl_vec = ampl;

	} ampl_vec_mutex_.unlock();

}

int main(int argc, char *argv[]) {

	return 0;

}