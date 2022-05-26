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

#include "mag_pl_detector/msg/powerline_poses_computation_result.hpp"

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

class PowerlinePositionsEstimatorNode : public rclcpp::Node {
public:
explicit
    PowerlinePositionsEstimatorNode(const std::string & node_name="pl_positions_estimator", 
                            const std::string & node_namespace="/pl_positions_estimator");

private:
    rclcpp::Subscription<mag_pl_detector::msg::PowerlinePosesComputationResult>::SharedPtr pl_poses_result_raw_sub_;

    rclcpp::Publisher<mag_pl_detector::msg::PowerlinePosesComputationResult>::SharedPtr pl_poses_result_est_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl0_pose_est_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl1_pose_est_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr odometry_timer_{nullptr};

	quat_t drone_quat_;
	quat_t last_drone_quat_;

    vector_t drone_pos_;
    vector_t last_drone_pos_;

    float kf_q_, kf_r_;

    vector_t pl0_position_est_;
    vector_t pl1_position_est_;

    bool has_updated_p0_;
    bool has_updated_p1_;

    float pl_current_est_;

    quat_t pl_dir_;

    plane_t projection_plane_;

    kf_est_t pl0_kf_[3];
    kf_est_t pl1_kf_[3];

    std::mutex kf_mutex_;
    std::mutex pl_positions_mutex_;
    std::mutex pl_dir_mutex_;
    std::mutex current_mutex_;

    float distance_discard_threshold_;

    void odometryCallback();
	void powerlinePosesRawCallback(mag_pl_detector::msg::PowerlinePosesComputationResult::SharedPtr msg);

	bool fetchDronePose();

	void kfPredict();
	void kfUpdate(std::vector<geometry_msgs::msg::Pose> poses);

	void update(kf_est_t *kf, vector_t new_vec);

	void publishPowerlinePosesEst();

};

int main(int argc, char *argv[]);