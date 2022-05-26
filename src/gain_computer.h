/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <queue>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "mag_pl_detector/msg/sine_reconstruction.hpp"

using namespace std::chrono_literals;

/*********************************************************************************
 * Class
 *********************************************************************************/

class GainComputerNode : public rclcpp::Node {
public:
explicit
    GainComputerNode(const std::string & node_name="gain_computer", 
                            const std::string & node_namespace="/gain_computer");

private:
	rclcpp::Subscription<mag_pl_detector::msg::SineReconstruction>::SharedPtr sine_reconstruction_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_gain_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_gain_pub_;

    int current_gain_ = 0;
    std::mutex current_gain_mutex_;

    int max_gain_step_;
    int initial_gain_step_;
    float gain_per_step_;
    float amplitude_range_;
    float target_norm_amplitude_;
    float hysteresis_upper_norm_threshold_;
    float hysteresis_lower_norm_threshold_;

    float target_amplitude_;
    float hysteresis_upper_threshold_;
    float hysteresis_lower_threshold_;

    void currentGainCallback(std_msgs::msg::Int32::SharedPtr msg);
    void sineReconstructionCallback(mag_pl_detector::msg::SineReconstruction::SharedPtr msg);

    int computeNewGain(float max_amplitude, int current_gain);
    void setGain(int new_gain);

};

/*********************************************************************************
 * Main
 *********************************************************************************/

int main(int argc, char *argv[]);