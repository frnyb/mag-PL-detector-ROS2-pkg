/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "mag_sample_file_publisher.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

MagSampleFilePublisherNode::MagSampleFilePublisherNode(const std::string & node_name, const std::string & node_namespace) 
				: Node(node_name, node_namespace) {
	
	this->declare_parameter<std::string>("filename", "~/mag_samples.txt");

	std::string filename;

	this->get_parameter("filename", filename);

	// RCLCPP_INFO(this->get_logger(), "Starting %s with parameters:%filename: %s%s%s",
	// 	node_name, std::endl, filename, std::endl, std::endl);

	msg_ = loadMsg(filename);

	mag_measurements_publisher_ = this->create_publisher<mag_pl_detector::msg::MagMeasurements>("mag_measurements", 10);

	timer_ = this->create_wall_timer(100ms, std::bind(&MagSampleFilePublisherNode::magTimerCallback, this));

}

void MagSampleFilePublisherNode::magTimerCallback() {

	mag_measurements_publisher_->publish(msg_);

}

mag_pl_detector::msg::MagMeasurements MagSampleFilePublisherNode::loadMsg(std::string filename) {

	filename_ = filename;

	std::ifstream fs;
	fs.open(filename);

	mag_pl_detector::msg::MagMeasurements msg;

	std::vector<mag_pl_detector::msg::MagMeasurement> samples;

	std::string line;

	while(std::getline(fs, line)) {

		mag_pl_detector::msg::MagMeasurement sub_msg;

		std::vector<int16_t> data;
		std::vector<int> time_offset;

		std::stringstream ss;
		ss.str(line);

		for (int i = 0; i < 12; i++) {
			
			std::string str_val;

			std::getline(ss, str_val, '\t');

			time_offset.push_back(std::stoi(str_val));

		}

		for (int i = 0; i < 12; i++) {

			std::string str_val;

			std::getline(ss, str_val, '\t');

			data.push_back(std::stoi(str_val));

		}

		sub_msg.data = data;
		sub_msg.time_offset = time_offset;

		samples.push_back(sub_msg);

	}

	msg.samples = samples;
	msg.count = samples.size();

	return msg;

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<MagSampleFilePublisherNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}