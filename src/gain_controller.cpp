/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "gain_controller.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

GainControllerNode::GainControllerNode(const std::string & node_name, const std::string & node_namespace) 
				: rclcpp::Node(node_name, node_namespace) {
	
	this->declare_parameter<int>("max_gain_step", 63);
	this->declare_parameter<int>("initial_gain_step", 0);
	this->declare_parameter<std::string>("control_gain_uio_ref_name", "controlgain");

	this->get_parameter("max_gain_step", max_gain_step_);
	this->get_parameter("initial_gain_step", initial_gain_step_);

	if (initial_gain_step_ > max_gain_step_) {
		initial_gain_step_ = max_gain_step_;
	} else if (initial_gain_step_ < 0) {
		initial_gain_step_ = 0;
	}

	std::string control_gain_ref_name;
	this->get_parameter("control_gain_uio_ref_name", control_gain_ref_name);

	XControlgain_Initialize(&xcg, control_gain_ref_name.c_str());

	current_gain_pub_ = this->create_publisher<std_msgs::msg::Int32>("current_gain", 10);

	target_gain_sub_ = this->create_subscription<std_msgs::msg::Int32>(
		"/gain_computer/target_gain", 10, std::bind(&GainControllerNode::targetGainCallback, this, std::placeholders::_1));

    gain_publish_timer_ = this->create_wall_timer(
      10ms, std::bind(&GainControllerNode::publishGains, this));

	setGain(initial_gain_step_);
	target_gain_ = initial_gain_step_;

}

GainControllerNode::~GainControllerNode() {

	XControlgain_Release(&xcg);

}

void GainControllerNode::targetGainCallback(std_msgs::msg::Int32::SharedPtr msg) {

	int new_gain = msg->data;

	if (new_gain != target_gain_) {

		setGain(new_gain);
		target_gain_ = new_gain;

	}
}

void GainControllerNode::setGain(int new_gain) {

	xcg_mutex_.lock(); {

		XControlgain_Set_gain_ref_in(&xcg, (uint32_t)new_gain);
		XControlgain_Start(&xcg);

	} xcg_mutex_.unlock();

}

void GainControllerNode::publishGains() {

	int current_gain = readCurrentGain();

	std_msgs::msg::Int32 current_gain_msg;
	current_gain_msg.data = current_gain;

	current_gain_pub_->publish(current_gain_msg);


}

int GainControllerNode::readCurrentGain() {

	int current_gain;

	xcg_mutex_.lock(); {

		XControlgain_Start(&xcg);
		current_gain = (int)XControlgain_Get_gain_out(&xcg);
		
	} xcg_mutex_.unlock();

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<GainControllerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}
