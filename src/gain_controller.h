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

#include "xcontrolgain.h"

using namespace std::chrono_literals;

/*********************************************************************************
 * Class
 *********************************************************************************/

class GainControllerNode : public rclcpp::Node {
public:
explicit
    GainControllerNode(const std::string & node_name="gain_controller", 
                            const std::string & node_namespace="/gain_controller");

    ~GainControllerNode();

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_gain_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_gain_pub_;

    rclcpp::TimerBase::SharedPtr gain_publish_timer_{nullptr};

    int max_gain_step_;
    int initial_gain_step_;

    int target_gain_;

    XControlgain xcg;
    std::mutex xcg_mutex_;

    void targetGainCallback(std_msgs::msg::Int32::SharedPtr msg);
    void setGain(int new_gain);

    void publishGains();
    int readCurrentGain();

};

/*********************************************************************************
 * Main
 *********************************************************************************/

int main(int argc, char *argv[]);