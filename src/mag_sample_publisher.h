/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "mag_sample_fetcher.h"
#include "mag_pl_detector/msg/mag_measurements.hpp"

using namespace std::chrono_literals;

/*********************************************************************************
 * Class
 *********************************************************************************/

class MagSamplePublisherNode : public rclcpp::Node {
public:
explicit
    MagSamplePublisherNode(const std::string & node_name="mag_sample_publisher", 
                            const std::string & node_namespace="/mag_sample_publisher");

private:
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<mag_pl_detector::msg::MagMeasurements>::SharedPtr mag_measurements_publisher_;

    mag_pl_detector::msg::MagMeasurements msg_;

    MagSampleFetcher *msf;

    int n_periods_;

    void magTimerCallback();

    mag_pl_detector::msg::MagMeasurements vecToMsg(std::vector<MagSample> vec);

};

/*********************************************************************************
 * Main
 *********************************************************************************/

int main(int argc, char *argv[]);