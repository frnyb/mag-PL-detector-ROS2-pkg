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

	this->declare_parameter<int>("max_n_samples", 800);
	this->get_parameter("max_n_samples", max_n_samples_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	fetchStaticTransforms();

    mag_measurements_sub_ = this->create_subscription<mag_pl_detector::msg::MagMeasurements>(
        "/mag_measurements", 
		10, std::bind(&SineReconstructorNode::magMeasurementsCallback, this, std::placeholders::_1));

	sine_reconstruction_pub_ = this->create_publisher<mag_pl_detector::msg::SineReconstruction>("sine_reconstruction", 10);

	mag_phasors_pub_ = this->create_publisher<mag_pl_detector::msg::MagneticPhasors3D>("mag_phasors", 10);

}

void SineReconstructorNode::magMeasurementsCallback(const mag_pl_detector::msg::MagMeasurements::SharedPtr msg) {

	MagMeasurementsClass mag_meas(*msg, R_drone_to_mags_, fixed_phase_, true, max_n_samples_);

	sine_reconstruction_pub_->publish(mag_meas.GetMsg());

	mag_pl_detector::msg::MagneticPhasors3D phasors_msg = mag_meas.GetPhasorsMsg(this->get_clock()->now());

	mag_phasors_pub_->publish(phasors_msg);

}

void SineReconstructorNode::fetchStaticTransforms() {

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


int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<SineReconstructorNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}