/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "mag_sample_publisher.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

MagSamplePublisherNode::MagSamplePublisherNode(const std::string & node_name, const std::string & node_namespace) 
				: rclcpp::Node(node_name, node_namespace) {
	
	this->declare_parameter<int>("bram_uio_number", 0);
	this->declare_parameter<int>("bram_size", 8192);
	this->declare_parameter<int>("n_periods", 20);

	int bram_uio_number;
	int bram_size;
	int n_periods;

	this->get_parameter("bram_uio_number", bram_uio_number);
	this->get_parameter("bram_size", bram_size);
	this->get_parameter("n_periods", n_periods);

	mag_measurements_publisher_ = this->create_publisher<mag_pl_detector::msg::MagMeasurements>("/mag_measurements", 10);

	msf = new MagSampleFetcher((unsigned int)bram_uio_number, (unsigned int)bram_size);

	timer_ = this->create_wall_timer(1ms, std::bind(&MagSamplePublisherNode::magTimerCallback, this));

	msf->GetSamples(n_periods);

	n_periods_ = n_periods;

}

void MagSamplePublisherNode::magTimerCallback() {

	std::vector<MagSample> samples = msf->GetSamples();

	mag_pl_detector::msg::MagMeasurements mag_measurements_msg = vecToMsg(samples);

	mag_measurements_publisher_->publish(mag_measurements_msg);

}

mag_pl_detector::msg::MagMeasurements MagSamplePublisherNode::vecToMsg(std::vector<MagSample> samples) {

	mag_pl_detector::msg::MagMeasurements msg;

	msg.count = samples.size();
	std::vector<mag_pl_detector::msg::MagMeasurement> vector;
	
	for (int i = 0; i < msg.count; i++) {

		mag_pl_detector::msg::MagMeasurement sub_msg;

		std::vector<int16_t> data;
		std::vector<int32_t> time_offsets;

		for (int j = 0; j < 12; j++) {
			sample_t sample = samples[i][j];
			data.push_back(sample.data);
			time_offsets.push_back(sample.time_offset);
		}

		sub_msg.data = data;
		sub_msg.time_offset = time_offsets;

		vector.push_back(sub_msg);

	}

	msg.samples = vector;

	return msg;

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<MagSamplePublisherNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}