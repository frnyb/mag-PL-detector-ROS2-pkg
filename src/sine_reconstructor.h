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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "mag_pl_detector/msg/mag_measurements.hpp"
#include "mag_pl_detector/msg/sine_reconstruction.hpp"
#include "mag_pl_detector/msg/magnetic_phasors3_d.hpp"

#include "mag_measurements_class.h"
#include "geometry.h"

/*********************************************************************************
 * Class
 *********************************************************************************/

class SineReconstructorNode : public rclcpp::Node {
public:
explicit
    SineReconstructorNode(const std::string & node_name="sine_reconstructor", 
                            const std::string & node_namespace="/sine_reconstructor");

private:
	const std::string mag_frame_names_[4] = {"mag0", "mag1", "mag2", "mag3"};

	rclcpp::Subscription<mag_pl_detector::msg::MagMeasurements>::SharedPtr mag_measurements_sub_;
	rclcpp::Publisher<mag_pl_detector::msg::SineReconstruction>::SharedPtr sine_reconstruction_pub_;

	rclcpp::Publisher<mag_pl_detector::msg::MagneticPhasors3D>::SharedPtr mag_phasors_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	rotation_matrix_t R_drone_to_mags_[4];
	vector_t v_drone_to_mags_[4];

	bool fixed_phase_;
	int max_n_samples_;

	void magMeasurementsCallback(const mag_pl_detector::msg::MagMeasurements::SharedPtr msg);

	void fetchStaticTransforms();

};

int main(int argc, char *argv[]);