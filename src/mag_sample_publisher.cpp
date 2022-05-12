/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "mag_sample_publisher.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

MagSamplePublisherNode::MagSamplePublisherNode(const std::string & node_name, const std::string & node_namespace) 
				: rclcpp::Node(node_name, node_namespace), sleep_rate(1000000) {
	
	this->declare_parameter<int>("bram_uio_number", 0);
	this->declare_parameter<int>("bram_size", 4*4096);
	this->declare_parameter<int>("n_periods", 20);

	int bram_uio_number;
	int bram_size;
	int n_periods;

	this->get_parameter("bram_uio_number", bram_uio_number);
	this->get_parameter("bram_size", bram_size);
	this->get_parameter("n_periods", n_periods);

    RCLCPP_INFO(this->get_logger(), "Starting %s with parameters: ", node_name);
	RCLCPP_INFO(this->get_logger(), "bram_uio_number: %d", bram_uio_number); 
	RCLCPP_INFO(this->get_logger(), "bram_size: %d", bram_size); 
	RCLCPP_INFO(this->get_logger(), "n_periods: %d", n_periods); 

	mag_measurements_publisher_ = this->create_publisher<mag_pl_detector::msg::MagMeasurements>("mag_measurements", 10);

	//msf = new SlidingWindowMagSampleFetcher((unsigned int)bram_uio_number, (unsigned int)bram_size);

	bram = new BRAM((unsigned int)bram_uio_number, (unsigned int)bram_size);
	n_periods_ = n_periods;

	//while(!msf->Start(n_periods)) {

	//	sleep_rate.sleep();

	//}

	first_run_ = true;

	fetch_samples_timer_ = this->create_wall_timer(10ms, std::bind(&MagSamplePublisherNode::fetchSamples, this));
	//publish_samples_timer_ = this->create_wall_timer(10ms, std::bind(&MagSamplePublisherNode::publishSamples, this));

}

void MagSamplePublisherNode::fetchSamples() {

	if (first_run_) {

		//while(!msf->Start()) {

		//	sleep_rate.sleep();

		//}

		bram[0] = 1;

		first_run_ = false;

	}

	//auto start_time = std::chrono::steady_clock::now();

	//while(!msf->Start()) {

	//	sleep_rate.sleep();

	//}

    //auto started_time = std::chrono::steady_clock::now();

	std::vector<MagSample> samples;
	
	//while(!msf->GetSamples(&samples)) {

	//	sleep_rate.sleep();

	//}

	while(bram[0] != 0) {

		sleep_rate.sleep();

	}

	for (int i = 0; i < N_SAMPLES; i++) {
		MagSample sample(bram, 1+i*12);

		samples.push_back(sample);
	}

    //auto finished_time = std::chrono::steady_clock::now();

    //std::chrono::duration<double> start_dur = started_time - start_time;
    //std::chrono::duration<double> finish_dur = finished_time - started_time;
    //std::chrono::duration<double> full_dur = finished_time - start_time;

    //std::cout <<  start_dur.count() << "\t" << finish_dur.count() << "\t" << full_dur.count() << std::endl;;

	//mag_samples_window_.push_back(samples);

	//if (mag_samples_window_.size() > n_periods_) {

	//	mag_samples_window_.erase(mag_samples_window_.begin());

	//}

	//while(!msf->Start()) {

	//	sleep_rate.sleep();

	//}

	bram[0] = 1;

	//publishSamples();

	mag_pl_detector::msg::MagMeasurements msg = vecToMsg(samples);
	mag_measurements_publisher_->publish(msg);

}

void MagSamplePublisherNode::publishSamples() {

	if (mag_samples_window_.size() < n_periods_) {

		return;

	}

	//mag_pl_detector::msg::MagMeasurements mag_measurements_msg = vecToMsg(samples);

	mag_pl_detector::msg::MagMeasurements mag_measurements_msg; // = windowToMsg();

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

mag_pl_detector::msg::MagMeasurements MagSamplePublisherNode::windowToMsg() {

	mag_pl_detector::msg::MagMeasurements msg;

	std::vector<mag_pl_detector::msg::MagMeasurement> vector;
	
	for (int i = 0; i < mag_samples_window_.size(); i++) {

		for (int j = 0; j < mag_samples_window_[i].size(); j++) {

			mag_pl_detector::msg::MagMeasurement sub_msg;

			std::vector<int16_t> data;
			std::vector<int32_t> time_offsets;

			for (int k = 0; k < 12; k++) {

				sample_t sample = mag_samples_window_[i][j][k];
				data.push_back(sample.data);
				time_offsets.push_back(sample.time_offset);

			}

			sub_msg.data = data;
			sub_msg.time_offset = time_offsets;

			vector.push_back(sub_msg);

		}
	}

	msg.samples = vector;
	msg.count = vector.size();

	return msg;

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<MagSamplePublisherNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}