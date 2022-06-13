/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "mag_pl_detector/msg/sine_reconstruction.hpp"
#include "mag_pl_detector/msg/magnetic_phasors3_d.hpp"

#include "geometry.h"
#include "bram_uio.h"

using namespace std::chrono_literals;

/*********************************************************************************
 * Class
 *********************************************************************************/

class SineReconstructionPublisherNode : public rclcpp::Node {
public:
explicit
    SineReconstructionPublisherNode(const std::string & node_name="sine_reconstruction_publisher", 
                            const std::string & node_namespace="/sine_reconstructor");

private:
	const std::string mag_frame_names_[4] = {"mag0", "mag1", "mag2", "mag3"};

    rclcpp::TimerBase::SharedPtr fetch_sines_timer_{nullptr};

	rclcpp::Publisher<mag_pl_detector::msg::SineReconstruction>::SharedPtr sine_reconstruction_pub_;
	rclcpp::Publisher<mag_pl_detector::msg::MagneticPhasors3D>::SharedPtr mag_phasors_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    BRAM *bram;
	rclcpp::Rate sleep_rate;
    bool first_run_;

	rotation_matrix_t R_drone_to_mags_[4];
	vector_t v_drone_to_mags_[4];

	void fetchStaticTransforms();

    void fetchSines();
    void publishSines(std::vector<float> amplitudes, std::vector<float> phases);

};

/*********************************************************************************
 * Main
 *********************************************************************************/

int main(int argc, char *argv[]);