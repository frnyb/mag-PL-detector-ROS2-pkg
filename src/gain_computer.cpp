/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "gain_computer.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

GainComputerNode::GainComputerNode(const std::string & node_name, const std::string & node_namespace) 
				: rclcpp::Node(node_name, node_namespace) {
	
	this->declare_parameter<int>("max_gain_step", 63);
	this->declare_parameter<int>("initial_gain_step", 0);
	this->declare_parameter<float>("gain_per_step", 7.);
	this->declare_parameter<float>("amplitude_range", 2048.);
	this->declare_parameter<float>("target_norm_amplitude", 0.5);
	this->declare_parameter<float>("hysteresis_upper_norm_threshold", 0.75);
	this->declare_parameter<float>("hysteresis_lower_norm_threshold", 0.25);
	this->declare_parameter<std::string>("control_gain_uio_ref_name", "controlgain");

	this->get_parameter("max_gain_step", max_gain_step_);
	this->get_parameter("initial_gain_step", initial_gain_step_);
	this->get_parameter("gain_per_step", gain_per_step_);
	this->get_parameter("amplitude_range", amplitude_range_);
	this->get_parameter("target_norm_amplitude", target_norm_amplitude_);
	this->get_parameter("hysteresis_upper_norm_threshold", hysteresis_upper_norm_threshold_);
	this->get_parameter("hysteresis_lower_norm_threshold", hysteresis_lower_norm_threshold_);

	if (initial_gain_step_ > max_gain_step_) {
		initial_gain_step_ = max_gain_step_;
	} else if (initial_gain_step_ < 0) {
		initial_gain_step_ = 0;
	}

	target_amplitude_ = target_norm_amplitude_ * amplitude_range_;
	hysteresis_lower_threshold_ = hysteresis_lower_norm_threshold_ * amplitude_range_;
	hysteresis_upper_threshold_ = hysteresis_upper_norm_threshold_ * amplitude_range_;

	target_gain_pub_ = this->create_publisher<std_msgs::msg::Int32>("target_gain", 10);

	current_gain_sub_ = this->create_subscription<std_msgs::msg::Int32>(
		"/gain_controller/current_gain", 10, std::bind(&GainComputerNode::currentGainCallback, this, std::placeholders::_1));

	sine_reconstruction_sub_ = this->create_subscription<mag_pl_detector::msg::SineReconstruction>(
		"/sine_reconstructor/sine_reconstruction", 10, std::bind(&GainComputerNode::sineReconstructionCallback, this, std::placeholders::_1));

	setGain(initial_gain_step_);

}

void GainComputerNode::currentGainCallback(std_msgs::msg::Int32::SharedPtr msg) {

	current_gain_mutex_.lock(); {

		current_gain_ = msg->data;

	} current_gain_mutex_.unlock();

}

void GainComputerNode::sineReconstructionCallback(mag_pl_detector::msg::SineReconstruction::SharedPtr msg) {

	std::vector<float> amplitudes = msg->amplitudes;

	float max_amplitude = -1;

	for (int i = 0; i < amplitudes.size(); i++) {

		float abs_amplitude = abs(amplitudes[i]);

		if (abs_amplitude > max_amplitude) {

			max_amplitude = abs_amplitude;

		}
	}

	int current_gain;
	current_gain_mutex_.lock(); {

		current_gain = current_gain_;

	}

	int new_gain = computeNewGain(max_amplitude, current_gain);
	setGain(new_gain);

}

int GainComputerNode::computeNewGain(float max_amplitude, int current_gain) {

	int new_gain_step = current_gain;

	if (max_amplitude < hysteresis_lower_threshold_ || max_amplitude > hysteresis_upper_threshold_) {

		float current_gain_val = gain_per_step_ * current_gain;
		float amplitude_raw = max_amplitude / current_gain_val;

		float gain_val_desired = target_amplitude_ / amplitude_raw;
		float gain_step_desired_f = gain_val_desired / gain_per_step_;

		new_gain_step = (int)gain_step_desired_f;

		if (new_gain_step < 0) {

			new_gain_step = 0;

		} else if (new_gain_step > max_gain_step_) {

			new_gain_step = max_gain_step_;

		}
	}

	return new_gain_step;

}

void GainComputerNode::setGain(int new_gain) {

	std_msgs::msg::Int32 target_gain_msg;
	target_gain_msg.data = new_gain;
	target_gain_pub_->publish(new_gain);

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<GainComputerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}
