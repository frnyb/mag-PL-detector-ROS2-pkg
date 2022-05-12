/*********************************************************************************
 * Includes
 *********************************************************************************/
#include <string>
#include <iostream>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>

#include "sliding_window_mag_sample_fetcher.h"
#include "mag_pl_detector/msg/mag_measurements.hpp"

using namespace std::chrono_literals;




#define N_PERIODS 10
#define N_SAMPLES_PER_PERIOD 8
#define N_SAMPLES N_PERIODS*N_SAMPLES_PER_PERIOD

/*********************************************************************************
 * Class
 *********************************************************************************/

class MagSamplePublisherNode : public rclcpp::Node {
public:
explicit
    MagSamplePublisherNode(const std::string & node_name="mag_sample_publisher", 
                            const std::string & node_namespace="/mag_sample_publisher");

private:
    rclcpp::TimerBase::SharedPtr fetch_samples_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr publish_samples_timer_{nullptr};

    rclcpp::Publisher<mag_pl_detector::msg::MagMeasurements>::SharedPtr mag_measurements_publisher_;

    mag_pl_detector::msg::MagMeasurements msg_;

    //SlidingWindowMagSampleFetcher *msf;
    BRAM *bram;
	rclcpp::Rate sleep_rate;
    bool first_run_;

    int n_periods_;

    std::vector<std::vector<MagSample>> mag_samples_window_;

    void fetchSamples();
    void publishSamples();

    mag_pl_detector::msg::MagMeasurements vecToMsg(std::vector<MagSample> vec);
    mag_pl_detector::msg::MagMeasurements windowToMsg();

};

/*********************************************************************************
 * Main
 *********************************************************************************/

int main(int argc, char *argv[]);