/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <vector>

#include <eigen3/Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mag_pl_detector/msg/mag_measurements.hpp"
#include "mag_pl_detector/msg/sine_reconstruction.hpp"
#include "mag_pl_detector/msg/magnetic_phasor.hpp"

#include "mag_measurements_class.h"

/*********************************************************************************
 * Class
 *********************************************************************************/

class SineReconstructorNode : public rclcpp::Node {
public:
explicit
    SineReconstructorNode(const std::string & node_name="sine_reconstructor", 
                            const std::string & node_namespace="/sine_reconstructor");

private:
	rclcpp::Subscription<mag_pl_detector::msg::MagMeasurements>::SharedPtr mag_measurements_sub_;
	rclcpp::Publisher<mag_pl_detector::msg::SineReconstruction>::SharedPtr sine_reconstruction_pub_;

	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag0_amplitude_vector_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag1_amplitude_vector_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag2_amplitude_vector_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag3_amplitude_vector_pub_;

	rclcpp::Publisher<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag0_phasor_pub_;
	rclcpp::Publisher<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag1_phasor_pub_;
	rclcpp::Publisher<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag2_phasor_pub_;
	rclcpp::Publisher<mag_pl_detector::msg::MagneticPhasor>::SharedPtr mag3_phasor_pub_;

	bool fixed_phase_;

	void magMeasurementsCallback(const mag_pl_detector::msg::MagMeasurements::SharedPtr msg);

};

int main(int argc, char *argv[]);