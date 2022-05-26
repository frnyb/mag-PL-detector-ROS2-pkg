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
//#include <cppopt/gauss_newton.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "mag_pl_detector/msg/magnetic_phasors3_d.hpp"
#include "mag_pl_detector/msg/powerline_poses_computation_result.hpp"

#include "cyclic_buffer.h"

#include "geometry.h"

using namespace std::chrono_literals;

/*********************************************************************************
 * Defines
 *********************************************************************************/

typedef struct measurement_item_t {
    std::vector<vector_t> W_vi;
    std::vector<vector_t> W_D_Mj_p;
    vector_t W_D_p;
};

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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl0_pose_raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl1_pose_raw_pub_;

    rclcpp::Subscription<mag_pl_detector::msg::PowerlinePosesComputationResult>::SharedPtr pl_poses_result_est_sub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr odometry_timer_{nullptr};

	rotation_matrix_t R_drone_to_mags_[4];
	vector_t v_drone_to_mags_[4];

    vector_t v_proj_mags_[4];
    vector_t v_proj_norm_ampls_[4];
    vector_t pl_unit_x_;

    plane_t projection_plane_;

	quat_t drone_quat_;
    vector_t drone_vec_;

    quat_t pl_dir_quat_;
    quat_t pl_dir_quat_hold_;

    std::mutex odometry_mutex_;
    std::mutex pl_dir_mutex_;
    std::mutex p_est_mutex_;

    std::vector<geometry_msgs::msg::Pose> pl_poses_;
    float pl_current_;

    vector_t D_vj_vecs_[4];
    vector_t D_vj_norm_vecs_[4];

    double I_;
    vector_t p1_;
    vector_t p2_;

    CyclicBuffer<measurement_item_t, 20> measurements_buffer;
    int buffer_size_;

    double I_max_;
    double cable_distance_max_;
    int max_LMA_iterations_;
    int max_weighting_iterations_;
    double lambda_first_step_;
    double lambda_increment_factor_;
    double lambda_decrement_factor_;
    double step_norm_success_threshold_;

    bool publish_p1_, publish_p2_;

    const double mu_0_ = 0.00000125663;
    const double T_per_LSB_ = 7.8125e-7;
    const double C_multiplier_ = mu_0_ / (2*M_PI);

	void powerlinePosesEstCallback(mag_pl_detector::msg::PowerlinePosesComputationResult::SharedPtr msg);
    void plDirCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odometryCallback();
    void magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg);

    void projectVectors(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors);
    void correctVectors(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors);

    void singleCableLinearComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors);
    void singleCableLinearBufferingComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors);
    void twoCablesNLLSComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors);
    void singleCableNLLSComputationMethod(std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors);

    bool levenbergMarquardt(Eigen::VectorXd &x);
    Eigen::MatrixXd J(const Eigen::VectorXd x);
    Eigen::VectorXd B(const Eigen::VectorXd x);

    bool levenbergMarquardtSingleCable(Eigen::VectorXd &x);
    Eigen::MatrixXd JSingleCable(const Eigen::VectorXd x);
    Eigen::VectorXd BSingleCable(const Eigen::VectorXd x);

    vector_t P_D_Mja_p_[4];
    Eigen::VectorXd P_v_;

    void publishPoseEstimationResult();

    void fetchStaticTransforms();

};


int main(int argc, char *argv[]);