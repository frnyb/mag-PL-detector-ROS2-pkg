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

#include "mag_pl_detector/msg/magnetic_phasors3_d.hpp"

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
	const std::string mag_frame_names_[4] = {"mag0", "mag1", "mag2", "mag3"};

	rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasors3D>::SharedPtr mag_phasors_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pub_;

	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag_norm_ampl_pubs_[4];

	bool fixed_phase_;

    //float q_, r_odom_, r_mag_vec_;

	float parallel_dot_prod_thresh_;

	vector_t mag_ampl_vecs_[4];
	vector_t norm_mag_ampl_vecs_[4];

	quat_t pl_dir_quat_;

	void getAmplitudes(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr phasors_msg);

	void computePowerlineDirection();

	void publishPowerlineDirection(quat_t q);
	void publishNormMagAmpVecs(vector_t vecs[4]);

	void magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg);

};

int main(int argc, char *argv[]);