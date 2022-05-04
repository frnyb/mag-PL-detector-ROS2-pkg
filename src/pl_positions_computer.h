/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <vector>
#include <mutex>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nlopt.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "mag_pl_detector/msg/magnetic_phasors3_d.hpp"
#include "mag_pl_detector/msg/powerline_poses_computation_result.hpp"

#include "geometry.h"

using namespace std::chrono_literals;

/*********************************************************************************
 * Defines
 *********************************************************************************/

typedef struct {
    vectord_t D_Mjalpha_p[4];
    Eigen::Matrix<double, 12, 1> D_V;
    vectord_t D_alpha_n_norm;
    double C;
    bool inverted;
    double epsilon;
    double conversion_factor;
} opt_data_t;

/*********************************************************************************
 * Class
 *********************************************************************************/

class PowerlinePositionsComputerNode : public rclcpp::Node {
public:
explicit
    PowerlinePositionsComputerNode(const std::string & node_name="pl_positions_computer", 
                            const std::string & node_namespace="/pl_positions_computer");
                            
    static double F(const std::vector<double> &x, std::vector<double> &grad, void *f_data);

private:
	const std::string mag_frame_names_[4] = {"mag0", "mag1", "mag2", "mag3"};

    rclcpp::Subscription<mag_pl_detector::msg::MagneticPhasors3D>::SharedPtr mag_phasors_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pl_dir_sub_;

    rclcpp::Publisher<mag_pl_detector::msg::PowerlinePosesComputationResult>::SharedPtr pl_poses_result_raw_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr odometry_timer_{nullptr};

	rotation_matrix_t R_drone_to_mags_[4];
	vector_t v_drone_to_mags_[4];

	quat_t drone_quat_;
    quat_t pl_dir_quat_;

    quat_t pl_dir_quat_holder_;

    std::mutex odometry_mutex_;
    std::mutex pl_dir_mutex_;

    nlopt::opt optimizer_;

    std::vector<double> x_;
    double opt_f_;

    const double mu_0_ = 0.00000125663;
    const double T_per_LSB_ = 7.8125e-7;

    double I_min_;
    double I_max_;
    double xyz_min_;
    double xyz_max_;

    std::vector<double> lb_;
    std::vector<double> ub_;

    opt_data_t opt_data_;

    void magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg);
    void plDirCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odometryCallback();

    void prepareOptimization(mag_pl_detector::msg::MagneticPhasors3D msg);
    void runOptimization();
    void publishPositions();

    void measVecSet(mag_pl_detector::msg::MagneticPhasors3D msg);
    void alphaSet(quat_t pl_dir_quat);
    void projMagPosSet();

	void fetchStaticTransforms();

};


int main(int argc, char *argv[]);