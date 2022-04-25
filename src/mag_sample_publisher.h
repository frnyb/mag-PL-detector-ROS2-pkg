/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "rclcpp.hpp"
#include "mag_sample_fetcher.h"

/*********************************************************************************
 * Class
 *********************************************************************************/

class MagSamplePublisherNode : rclcpp::Node {
public:
    MagSampleFetcherNode(const std::string & node_name="mag_sample_fetcher", const std::string & node_namespace="/mag_sample_fetcher");

private:
    rclcpp::TimerBase::SharedPtr drone_tf_timer_{nullptr};

    MagSampleFetcher msf;

    void magTimerCallback();

};