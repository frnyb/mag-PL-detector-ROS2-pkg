/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <queue>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include "mag_pl_detector/msg/sine_reconstruction.hpp"

#include "xsamplecnttargetcontroller.h"

using namespace std::chrono_literals;

/*********************************************************************************
 * Class
 *********************************************************************************/

class SampleFrequencyControllerNode : public rclcpp::Node {
public:
explicit
    SampleFrequencyControllerNode(const std::string & node_name="sample_frequency_controller", 
                            const std::string & node_namespace="/sample_frequency_controller");

    ~SampleFrequencyControllerNode();

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_sample_frequency_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_clock_divisor_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sample_frequency_sub_;

    rclcpp::TimerBase::SharedPtr publish_timer_{nullptr};

    float max_sample_frequency_;
    float min_sample_frequency_;

    float clock_frequency_;

    int clock_divisor_;
    float sample_frequency_;
    std::mutex values_mutex_;

    XSamplecnttargetcontroller xsctc;
    std::mutex xsctc_mutex_;

    void targetFrequencyCallback(std_msgs::msg::Float32::SharedPtr msg);
    void setSampleFrequency(float new_frequency);

    void publishSampleFrequency();

};

/*********************************************************************************
 * Main
 *********************************************************************************/

int main(int argc, char *argv[]);