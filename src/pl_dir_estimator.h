/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <vector>
#include <mutex>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "geometry.h"

using namespace std::chrono_literals;

/*****************************************************************************/
// Defines
/*****************************************************************************/

typedef struct {

    float state_est;
    float var_est;

} kf_est_t;

/*********************************************************************************
 * Class
 *********************************************************************************/

class PowerlineDirectionEstimatorNode : public rclcpp::Node {
public:
explicit
    PowerlineDirectionEstimatorNode(const std::string & node_name="pl_dir_estimator", 
                            const std::string & node_namespace="/pl_dir_estimator");

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_est_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr odometry_timer_{nullptr};

	bool fixed_phase_;

	quat_t drone_quat_;
	quat_t last_drone_quat_;

    float kf_q_, kf_r_;

    quat_t pl_direction_est_;

    kf_est_t pl_yaw_est_;
    kf_est_t pl_pitch_est_;

    std::mutex direction_mutex_;
    std::mutex kf_mutex_;

    void odometryCallback();
	void powerlineDirectionRawCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

	void kfPredict();
	void kfUpdate(quat_t q);

	kf_est_t update(kf_est_t old_est, float new_x);

	bool fetchDroneOrientation();

    float backmapAngle(float angle);
    float mapAngle(float curr_angle, float new_angle);

	void publishPowerlineDirectionEst();

};

int main(int argc, char *argv[]);