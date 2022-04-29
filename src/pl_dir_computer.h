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

#include "mag_pl_detector/msg/magnetic_phasor.hpp"

#include "geometry.h"

using namespace std::chrono_literals;

/*********************************************************************************
 * Class
 *********************************************************************************/

class PowerlineDirectionComputerNode : public rclcpp::Node {
public:
explicit
    PowerlineDirectionComputerNode(const std::string & node_name="pl_dir_computer", 
                            const std::string & node_namespace="/pl_dir_computer");

private:
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag0_amplitude_vector_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag1_amplitude_vector_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag2_amplitude_vector_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag3_amplitude_vector_sub_;

	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag0_phasor_sub_;
	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag1_phasor_sub_;
	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag2_phasor_sub_;
	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag3_phasor_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr mag_vector_timer_{nullptr};

	bool fixed_phase_;

	rotation_matrix_t R_drone_to_mag0_;
	rotation_matrix_t R_drone_to_mag1_;
	rotation_matrix_t R_drone_to_mag2_;
	rotation_matrix_t R_drone_to_mag3_;

	vector_t v_drone_to_mag0_;
	vector_t v_drone_to_mag1_;
	vector_t v_drone_to_mag2_;
	vector_t v_drone_to_mag3_;

    float q_, r_odom_, r_mag_vec_;

	float parallel_dot_prod_thresh_;

	vector_t mag0_ampl_vec;
	vector_t mag1_ampl_vec;
	vector_t mag2_ampl_vec;
	vector_t mag3_ampl_vec;

	vector_t norm_mag0_ampl_vec;
	vector_t norm_mag1_ampl_vec;
	vector_t norm_mag2_ampl_vec;
	vector_t norm_mag3_ampl_vec;

	std::mutex ampl_vec_mutex_;

	void updateFromMagVectors();

	void publishPowerlineDirection(quat_t q);

	void mag0AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
	void mag1AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
	void mag2AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
	void mag3AmplitudeCallback(geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

	void mag0PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg);
	void mag1PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg);
	void mag2PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg);
	void mag3PhasorCallback(mag_pl_detector::msg::MagneticPhasor::SharedPtr msg);

};

int main(int argc, char *argv[]);