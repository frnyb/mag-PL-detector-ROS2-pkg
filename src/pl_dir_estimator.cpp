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

	this->declare_parameter<float>("kf_q", 0.999);
	this->get_parameter("kf_q", kf_q_);

	this->declare_parameter<float>("kf_r", 1-kf_q_);
	this->get_parameter("kf_r", kf_r_);

	pl_direction_raw_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/pl_dir_computer/powerline_direction_raw", 10, std::bind(&PowerlineDirectionEstimatorNode::powerlineDirectionRawCallback, this, std::placeholders::_1));
	
	pl_direction_est_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>( "powerline_direction_est", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odometry_timer_ = this->create_wall_timer(
      10ms, std::bind(&PowerlineDirectionEstimatorNode::odometryCallback, this));

}

void PowerlineDirectionEstimatorNode::odometryCallback() {

	if (fetchDroneOrientation()) {

		kfPredict();

		publishPowerlineDirectionEst();

	}
}

void PowerlineDirectionEstimatorNode::powerlineDirectionRawCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {

	quat_t pl_q(
		msg->pose.orientation.w,
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z
	);

	kfUpdate(pl_q);

}

void PowerlineDirectionEstimatorNode::kfPredict() {

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
        pl_yaw_est_.var_est += kf_q_;

        pl_pitch_est_.state_est = pl_pitch_est_.state_est + delta_pitch;
        pl_pitch_est_.var_est += kf_q_;

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

        pl_direction_est_ = pl_dir;

    } direction_mutex_.unlock();

}

void PowerlineDirectionEstimatorNode::kfUpdate(quat_t q) {

	orientation_t eul = quatToEul(q);

	kf_mutex_.lock(); {

		pl_pitch_est_ = update(pl_pitch_est_, eul(1));
		pl_yaw_est_ = update(pl_yaw_est_, eul(2));

		eul(0) = 0;
		eul(1) = pl_pitch_est_.state_est;
		eul(2) = pl_yaw_est_.state_est;

	} kf_mutex_.unlock();

	q = eulToQuat(eul);

	direction_mutex_.lock(); {

		pl_direction_est_ = q;

	} direction_mutex_.unlock();

}

kf_est_t PowerlineDirectionEstimatorNode::update(kf_est_t old_est, float new_x) {

		float new_angle = mapAngle(old_est.state_est, new_x);

		float y_bar = new_angle - old_est.state_est;
		float s = old_est.var_est + kf_r_;

		float k = old_est.var_est / s;

		old_est.state_est += k*y_bar;
		old_est.var_est *= 1-k;

		old_est.state_est = backmapAngle(old_est.state_est);

		return old_est;

}

bool PowerlineDirectionEstimatorNode::fetchDroneOrientation() {

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

	return true;

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

void PowerlineDirectionEstimatorNode::publishPowerlineDirectionEst() {

	quat_t q;

	direction_mutex_.lock(); {

		q = pl_direction_est_;

	} direction_mutex_.unlock();

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

	pl_direction_est_pub_->publish(msg);

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<PowerlineDirectionEstimatorNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}