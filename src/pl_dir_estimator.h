/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <vector>
#include <mutex>

#include <eigen3/Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "mag_pl_detector/msg/magnetic_phasor.hpp"

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
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag0_amplitude_vector_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag1_amplitude_vector_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag2_amplitude_vector_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag3_amplitude_vector_sub_;

	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag0_phasor_sub_;
	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag1_phasor_sub_;
	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag2_phasor_sub_;
	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag3_phasor_sub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr odometry_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr mag_vector_timer_{nullptr};

	bool fixed_phase_;

	quat_t drone_quat_;
	quat_t last_drone_quat_;

	geometry_msgs::msg::

    float q_, r_odom_, r_mag_vec_;

	float parallel_dot_prod_thresh_;

    quat_t pl_direction_;

    kf_est_t pl_yaw_est_;
    kf_est_t pl_pitch_est_;

	vector_t mag0_ampl_vec;
	vector_t mag1_ampl_vec;
	vector_t mag2_ampl_vec;
	vector_t mag3_ampl_vec;

    std::mutex direction_mutex_;
    std::mutex kf_mutex_;
    std::mutex odometry_mutex_;
	std::mutex ampl_vec_mutex_;

	void predict();
	kf_est_t update(float r, kf_est_t est_old, float x_new);
	void updateFromOdometry();
	void updateFromMagVectors();

    float mapAngle(float curr_angle, float new_angle);
    float backmapAngle(float angle);

	void publishPowerlineDirection(quat_t q);
	void publishMagPowerlineDirection(quat_t q);

    void odometryCallback();

	void mag0AmplitudeCallback(geometry_msgs::msg::Vector3Stamped);
	void mag1AmplitudeCallback(geometry_msgs::msg::Vector3Stamped);
	void mag2AmplitudeCallback(geometry_msgs::msg::Vector3Stamped);
	void mag3AmplitudeCallback(geometry_msgs::msg::Vector3Stamped);

	void mag0PhasorCallback(mag_pl_detector::msg::MagneticPhasor);
	void mag1PhasorCallback(mag_pl_detector::msg::MagneticPhasor);
	void mag2PhasorCallback(mag_pl_detector::msg::MagneticPhasor);
	void mag3PhasorCallback(mag_pl_detector::msg::MagneticPhasor);

};

int main(int argc, char *argv[]);