/*********************************************************************************
 * Includes
 *********************************************************************************/
#include "sine_reconstruction_publisher.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

SineReconstructionPublisherNode::SineReconstructionPublisherNode(const std::string & node_name, const std::string & node_namespace) 
				: rclcpp::Node(node_name, node_namespace), sleep_rate(1000000) {
	
	this->declare_parameter<int>("bram_uio_number", 1);
	this->declare_parameter<int>("bram_size", 8192);

	int bram_uio_number;
	int bram_size;

	this->get_parameter("bram_uio_number", bram_uio_number);
	this->get_parameter("bram_size", bram_size);

	RCLCPP_INFO(this->get_logger(), "Starting %s with parameters: ", node_name);
	RCLCPP_INFO(this->get_logger(), "bram_uio_number: %d", bram_uio_number); 
	RCLCPP_INFO(this->get_logger(), "bram_size: %d", bram_size); 

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	fetchStaticTransforms();

	sine_reconstruction_pub_ = this->create_publisher<mag_pl_detector::msg::SineReconstruction>("sine_reconstruction", 10);
	mag_phasors_pub_ = this->create_publisher<mag_pl_detector::msg::MagneticPhasors3D>("mag_phasors", 10);

	bram = new BRAM((unsigned int)bram_uio_number, (unsigned int)bram_size);
	first_run_ = true;

	fetch_sines_timer_ = this->create_wall_timer(10ms, std::bind(&SineReconstructionPublisherNode::fetchSines, this));

}

void SineReconstructionPublisherNode::fetchStaticTransforms() {

	for (int i = 0; i < 4; i++) {

		geometry_msgs::msg::TransformStamped mag_tf;

		while(true) {

			try {

				mag_tf = tf_buffer_->lookupTransform("drone", mag_frame_names_[i], tf2::TimePointZero);

				break;

			} catch(tf2::TransformException & ex) {

				//RCLCPP_INFO(this->get_logger(), "Could not get mag transform, trying again...");

			}
		}

		quat_t mag_quat(
			mag_tf.transform.rotation.w,
			mag_tf.transform.rotation.x,
			mag_tf.transform.rotation.y,
			mag_tf.transform.rotation.z
		);

		R_drone_to_mags_[i] = quatToMat(mag_quat);

		v_drone_to_mags_[i](0) = mag_tf.transform.translation.x;
		v_drone_to_mags_[i](1) = mag_tf.transform.translation.x;
		v_drone_to_mags_[i](2) = mag_tf.transform.translation.x;

	}
}

void SineReconstructionPublisherNode::fetchSines() {

	if (first_run_) {

		(*bram)[0] = 1;

		first_run_ = false;

	}

	while((*bram)[0] != 0) {

		sleep_rate.sleep();

	}

	std::vector<float> amplitudes;
	std::vector<float> phases;

	for (int i = 0; i < 12; i++) {

		uint32_t amp_val = (*bram)[1+i];
		uint32_t phase_val = (*bram)[13+i];

		float amplitude = *((float *)amp_val);
		float phase = *((float *)phase_val);

		amplitudes.push_back(amplitude);
		phases.push_back(phase);

	}

	(*bram)[0] = 1;

	publishSines(amplitudes, phases);

}

void SineReconstructionPublisherNode::publishSines(std::vector<float> amplitudes, std::vector<float> phases) {

	rclcpp::Time stamp = this->get_clock()->now();

	mag_pl_detector::msg::SineReconstruction sine_recon_msg;
	sine_recon_msg.amplitudes = amplitudes;
	sine_recon_msg.phases = phases;

	mag_pl_detector::msg::MagneticPhasors3D phasors_msg;
	std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors_vector;

	for (int i = 0; i < 4; i++) {

		vector_t ampl(
			amplitudes[i*3],
			amplitudes[i*3+1],
			amplitudes[i*3+2]
		);

		ampl = R_drone_to_mags_[i] * ampl;

		vector_t norm_ampl = ampl / ampl.norm();

		mag_pl_detector::msg::MagneticPhasor3D mag_msg;
		mag_msg.header.frame_id = "drone";
		mag_msg.header.stamp = stamp;

		mag_msg.amplitudes.x = ampl(0);
		mag_msg.amplitudes.y = ampl(1);
		mag_msg.amplitudes.z = ampl(2);

		mag_msg.normalized_amplitudes.x = norm_ampl(0);
		mag_msg.normalized_amplitudes.y = norm_ampl(1);
		mag_msg.normalized_amplitudes.z = norm_ampl(2);

		mag_msg.phases.x = phases[i*3];
		mag_msg.phases.y = phases[i*3+1];
		mag_msg.phases.z = phases[i*3+2];

		phasors_vector.push_back(mag_msg);

	}

	phasors_msg.phasors = phasors_vector;

	sine_reconstruction_pub_->publish(sine_recon_msg);
	mag_phasors_pub_->publish(phasors_msg);

}


int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<SineReconstructionPublisherNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}