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

#include "mag_pl_detector/msg/magnetic_phasors3_d.hpp"

#include "cyclic_buffer.h"

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

	rotation_matrix_t R_drone_to_mags_[4];
	vector_t v_drone_to_mags_[4];

	std::mutex odometry_mutex_;

	quat_t drone_quat_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr odometry_timer_{nullptr};

	float parallel_dot_prod_thresh_;

	vector_t mag_ampl_vecs_[4];
	vector_t norm_mag_ampl_vecs_[4];

	CyclicBuffer<std::vector<vector_2D_t>, 20> points_buffer;

	quat_t pl_dir_quat_;

    void odometryCallback();

	void getAmplitudes(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr phasors_msg);

	void computePowerlineDirection();

	quat_t crossVectorMeanDirectionComputation();
	quat_t projectionVectorMeanDirectionComputation();
	quat_t projectionLineFitDirectionComputation();
	quat_t projectionBufferLineFitDirectionComputation();

	void publishPowerlineDirection(quat_t q);
	void publishNormMagAmpVecs(vector_t vecs[4]);

	void magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg);

	void correctVectors();

    void fetchStaticTransforms();

};

int main(int argc, char *argv[]);