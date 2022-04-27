/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "mag_pl_detector/msg/mag_measurements.hpp"

using namespace std::chrono_literals;

/*********************************************************************************
 * Class
 *********************************************************************************/

class MagSampleFilePublisherNode : public rclcpp::Node {
public:
explicit
    MagSampleFilePublisherNode(const std::string & node_name="mag_sample_publisher", 
                            const std::string & node_namespace="/mag_sample_publisher");

private:
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<mag_pl_detector::msg::MagMeasurements>::SharedPtr mag_measurements_publisher_;

    mag_pl_detector::msg::MagMeasurements msg_;
    std::string filename_;

    void magTimerCallback();

    mag_pl_detector::msg::MagMeasurements loadMsg(std::string filename);

};

/*********************************************************************************
 * Main
 *********************************************************************************/

int main(int argc, char *argv[]);