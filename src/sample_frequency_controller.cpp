/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "sample_frequency_controller.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

SampleFrequencyControllerNode::SampleFrequencyControllerNode(const std::string & node_name, const std::string & node_namespace) 
				: rclcpp::Node(node_name, node_namespace) {
	
	this->declare_parameter<float>("max_sample_frequency", 333333);
	this->declare_parameter<float>("min_sample_frequency", 20833);
	this->declare_parameter<float>("initial_sample_frequency", 20833);
	this->declare_parameter<float>("clock_frequency", 100000000);
	this->declare_parameter<std::string>("sample_cnt_target_controller_uio_ref_name", "samplecnttargetcon");

	this->get_parameter("max_sample_frequency", max_sample_frequency_);
	this->get_parameter("min_sample_frequency", min_sample_frequency_);
	this->get_parameter("initial_sample_frequency", sample_frequency_);
	this->get_parameter("clock_frequency", clock_frequency_);

	if (sample_frequency_ > max_sample_frequency_) {
		sample_frequency_ = max_sample_frequency_;
	} else if (sample_frequency_ < min_sample_frequency_) {
		sample_frequency_ = min_sample_frequency_;
	}

	std::string ip_ref_name;
	this->get_parameter("sample_cnt_target_controller_uio_ref_name", ip_ref_name);

	XSamplecnttargetcontroller_Initialize(&xsctc, ip_ref_name.c_str());

	current_sample_frequency_pub_ = this->create_publisher<std_msgs::msg::Float32>("current_sample_frequency", 10);
	current_clock_divisor_pub_ = this->create_publisher<std_msgs::msg::Int32>("current_clock_divisor", 10);

	target_sample_frequency_sub_ = this->create_subscription<std_msgs::msg::Float32>(
		"target_sample_frequency", 10, std::bind(&SampleFrequencyControllerNode::targetFrequencyCallback, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(
      500ms, std::bind(&SampleFrequencyControllerNode::publishSampleFrequency, this));

	setSampleFrequency(sample_frequency_);

}

SampleFrequencyControllerNode::~SampleFrequencyControllerNode() {

	XSamplecnttargetcontroller_Release(&xsctc);

}

void SampleFrequencyControllerNode::targetFrequencyCallback(std_msgs::msg::Float32::SharedPtr msg) {

	float new_freq = msg->data;

	setSampleFrequency(new_freq);

}

void SampleFrequencyControllerNode::setSampleFrequency(float new_frequency) {

	if (new_frequency > max_sample_frequency_) {

		new_frequency = max_sample_frequency_;

	} else if (new_frequency < min_sample_frequency_) {

		new_frequency = min_sample_frequency_;

	}

	int clock_div = (int)(clock_frequency_ / (new_frequency * 3.));

	values_mutex_.lock(); {

		clock_divisor_ = clock_div;
		sample_frequency_ = new_frequency;

	} values_mutex_.unlock();

	xsctc_mutex_.lock(); {

		XSamplecnttargetcontroller_Set_sample_cnt_target_in(&xsctc, (uint32_t)clock_divisor_);
		XSamplecnttargetcontroller_Start(&xsctc);

	} xsctc_mutex_.unlock();

}

void SampleFrequencyControllerNode::publishSampleFrequency() {

	int clock_div;
	float sample_freq;

	values_mutex_.lock(); {

		clock_div = clock_divisor_;
		sample_freq = sample_frequency_;

	} values_mutex_.unlock();

	std_msgs::msg::Int32 clock_div_msg;
	clock_div_msg.data = clock_div;

	current_clock_divisor_pub_->publish(clock_div_msg);

	std_msgs::msg::Float32 sample_frequency_msg;
	sample_frequency_msg.data = sample_freq;

	current_sample_frequency_pub_->publish(sample_frequency_msg);

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<SampleFrequencyControllerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}
