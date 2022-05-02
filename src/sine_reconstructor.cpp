/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "sine_reconstructor.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

SineReconstructorNode::SineReconstructorNode(const std::string & node_name, 
								const std::string & node_namespace) :
					Node(node_name, node_namespace) {
	
	this->declare_parameter<bool>("fixed_phase", true);

	this->get_parameter("fixed_phase", fixed_phase_);

    mag_measurements_sub_ = this->create_subscription<mag_pl_detector::msg::MagMeasurements>(
        "/mag_measurements", 
		10, std::bind(&SineReconstructorNode::magMeasurementsCallback, this, std::placeholders::_1));

	sine_reconstruction_pub_ = this->create_publisher<mag_pl_detector::msg::SineReconstruction>("/sine_reconstruction", 10);

	if (fixed_phase_) {

		mag0_amplitude_vector_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/mag0_amplitude_vector", 10);
		mag1_amplitude_vector_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/mag1_amplitude_vector", 10);
		mag2_amplitude_vector_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/mag2_amplitude_vector", 10);
		mag3_amplitude_vector_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/mag3_amplitude_vector", 10);

	} else {

		mag0_phasor_pub_ = this->create_publisher<mag_pl_detector::msg::MagneticPhasor>("/mag0_phasor", 10);
		mag1_phasor_pub_ = this->create_publisher<mag_pl_detector::msg::MagneticPhasor>("/mag1_phasor", 10);
		mag2_phasor_pub_ = this->create_publisher<mag_pl_detector::msg::MagneticPhasor>("/mag2_phasor", 10);
		mag3_phasor_pub_ = this->create_publisher<mag_pl_detector::msg::MagneticPhasor>("/mag3_phasor", 10);

	}

}

void SineReconstructorNode::magMeasurementsCallback(const mag_pl_detector::msg::MagMeasurements::SharedPtr msg) {

	MagMeasurementsClass mag_meas(*msg, fixed_phase_, true);

	sine_reconstruction_pub_->publish(mag_meas.GetMsg());

	if (fixed_phase_) {

		std::vector<geometry_msgs::msg::Vector3Stamped> ampl_msgs = mag_meas.GetAmplitudeVectorMsgs(this->get_clock()->now());

		mag0_amplitude_vector_pub_->publish(ampl_msgs[0]);
		mag1_amplitude_vector_pub_->publish(ampl_msgs[1]);
		mag2_amplitude_vector_pub_->publish(ampl_msgs[2]);
		mag3_amplitude_vector_pub_->publish(ampl_msgs[3]);

	} else {

		std::vector<mag_pl_detector::msg::MagneticPhasor> phasor_msgs = mag_meas.GetPhasorMsgs(this->get_clock()->now());

		mag0_phasor_pub_->publish(phasor_msgs[0]);
		mag1_phasor_pub_->publish(phasor_msgs[1]);
		mag2_phasor_pub_->publish(phasor_msgs[2]);
		mag3_phasor_pub_->publish(phasor_msgs[3]);

	}

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<SineReconstructorNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}