/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "pl_dir_computer.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

PowerlineDirectionComputerNode::PowerlineDirectionComputerNode(const std::string & node_name, 
                            const std::string & node_namespace) : rclcpp::Node(node_name, node_namespace) {


	this->declare_parameter<bool>("fixed_phase", true);
	this->get_parameter("fixed_phase", fixed_phase_);

	//this->declare_parameter<float>("kf_q", 0.5);
	//this->get_parameter("kf_q", q_);

	//this->declare_parameter<float>("kf_r_odom", 1-q_);
	//this->get_parameter("kf_r_odom", r_odom_);

	//this->declare_parameter<float>("kf_r_mag_vec", 1-q_);
	//this->get_parameter("kf_r_mag_vec", r_mag_vec_);

	this->declare_parameter<float>("vector_parallel_dot_prod_thresh", 0.95);
	this->get_parameter("vector_parallel_dot_prod_thresh", parallel_dot_prod_thresh_);

    RCLCPP_INFO(this->get_logger(), "Starting %s with parameters:%fixed_phase: %s %svector_parallel_dot_prod_thresh: %f %s%s",
		node_name, std::endl, std::to_string(fixed_phase_), std::endl, parallel_dot_prod_thresh_, std::endl, std::endl);

	mag_phasors_sub_ = this->create_subscription<mag_pl_detector::msg::MagneticPhasors3D>(
		"/sine_reconstructor/mag_phasors", 10, std::bind(&PowerlineDirectionComputerNode::magPhasorsCallback, this, std::placeholders::_1));

	pl_direction_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("powerline_direction_raw", 10);

	const std::string mag_norm_amp_topics[4] = {
		"mag0_norm_mag_ampl_vector",
		"mag1_norm_mag_ampl_vector",
		"mag2_norm_mag_ampl_vector",
		"mag3_norm_mag_ampl_vector"
	};

	for (int i = 0; i < 4; i++) {

		mag_norm_ampl_pubs_[i] = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(mag_norm_amp_topics[i], 10);

	}

}

void PowerlineDirectionComputerNode::computePowerlineDirection() {

	vector_t cross_vec_sum(0,0,0);
	int cnt = 0;

	//for (int i = 0; i < 3; i++) {
	for (int i = 0; i < 2; i++) {

		//for (int j = i+1; j < 4; j++) {
		for (int j = i+1; j < 3; j++) {

			if (abs(norm_mag_ampl_vecs_[i].dot(norm_mag_ampl_vecs_[j])) > parallel_dot_prod_thresh_) {

				continue;

			}

			vector_t cross_vec = norm_mag_ampl_vecs_[i].cross(norm_mag_ampl_vecs_[j]);
			cross_vec /= cross_vec.norm();

			if (cnt > 0) {

				if (cross_vec_sum.dot(cross_vec) < 0) {

					cross_vec = -cross_vec;

				}
			}

			cross_vec_sum += cross_vec;
			cnt++;

		}
	}

	if (cnt == 0) {

		return;

	}

	vector_t pl_dir = cross_vec_sum / cnt;
	pl_dir /= pl_dir.norm();

	quat_t q;

	vector_t unit_x(1,0,0);
	vector_t a = pl_dir.cross(unit_x);

	q(0) = unit_x.dot(pl_dir);
	q(1) = a(0);
	q(2) = a(1);
	q(3) = a(2);
	q /= q.norm();

	pl_dir_quat_ = q;

}

void PowerlineDirectionComputerNode::publishPowerlineDirection(quat_t q) {

	geometry_msgs::msg::PoseStamped msg;

	msg.header.frame_id = "drone";
	msg.header.stamp = this->get_clock()->now();

	msg.pose.orientation.w = q(0);
	msg.pose.orientation.x = q(1);
	msg.pose.orientation.y = q(2);
	msg.pose.orientation.z = q(3);

	msg.pose.position.x = 0;
	msg.pose.position.y = 0;
	msg.pose.position.z = 0;

	pl_direction_pub_->publish(msg);

}

void PowerlineDirectionComputerNode::publishNormMagAmpVecs(vector_t vecs[4]) {

	rclcpp::Time stamp = this->get_clock()->now();

	for (int i = 0; i < 4; i++) {

		geometry_msgs::msg::Vector3Stamped msg;

		msg.header.frame_id = mag_frame_names_[i];
		msg.header.stamp = stamp;

		msg.vector.x = norm_mag_ampl_vecs_[i](0);
		msg.vector.y = norm_mag_ampl_vecs_[i](1);
		msg.vector.z = norm_mag_ampl_vecs_[i](2);

		mag_norm_ampl_pubs_[i]->publish(msg);

	}
}

void PowerlineDirectionComputerNode::getAmplitudes(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr phasors_msg) {

	for (int i = 0; i < 4; i++) {

		mag_pl_detector::msg::MagneticPhasor3D phasor_msg = phasors_msg->phasors[i];

		vector_t ampl(
			phasor_msg.amplitudes.x,
			phasor_msg.amplitudes.y,
			phasor_msg.amplitudes.z
		);

		vector_t norm_ampl(
			phasor_msg.normalized_amplitudes.x,
			phasor_msg.normalized_amplitudes.y,
			phasor_msg.normalized_amplitudes.z
		);

		mag_ampl_vecs_[i] = ampl;
		norm_mag_ampl_vecs_[i] = norm_ampl;

	}
}

void PowerlineDirectionComputerNode::magPhasorsCallback(mag_pl_detector::msg::MagneticPhasors3D::SharedPtr msg) {

	getAmplitudes(msg);
	computePowerlineDirection();

	publishPowerlineDirection(pl_dir_quat_);
	publishNormMagAmpVecs(norm_mag_ampl_vecs_);

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<PowerlineDirectionComputerNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}