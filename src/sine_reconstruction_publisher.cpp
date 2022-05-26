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
	this->declare_parameter<int>("bram_size", 4096);

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

    pl_dir_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pl_dir_estimator/powerline_direction_est", 10,
        std::bind(&SineReconstructionPublisherNode::plDirCallback, this, std::placeholders::_1)
    );

	sine_reconstruction_pub_ = this->create_publisher<mag_pl_detector::msg::SineReconstruction>("sine_reconstruction", 10);
	mag_phasors_pub_ = this->create_publisher<mag_pl_detector::msg::MagneticPhasors3D>("mag_phasors", 10);

	bram = new BRAM((unsigned int)bram_uio_number, (unsigned int)bram_size);
	first_run_ = true;

	fetch_sines_timer_ = this->create_wall_timer(10ms, std::bind(&SineReconstructionPublisherNode::fetchSines, this));

	RCLCPP_INFO(this->get_logger(), "Running SineReconstructionPublisher"); 

}

void SineReconstructionPublisherNode::plDirCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    //std::cout << "plDirCallback" << std::endl;

    quat_t q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );

    if (pl_dir_mutex_.try_lock()) {

        pl_dir_quat_ = q;

        pl_dir_mutex_.unlock();

    }
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

		float amplitude = *((float *)&amp_val);
		float phase = *((float *)&phase_val);

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

	std::vector<vector_t> amplitude_vecs;

	for (int i = 0; i < 4; i++) {

		vector_t ampl(
			amplitudes[i*3],
			amplitudes[i*3+1],
			-amplitudes[i*3+2]
		);

		ampl = R_drone_to_mags_[i] * ampl;

		amplitude_vecs.push_back(ampl);

	}

	amplitude_vecs = correctAmplitudes(amplitude_vecs);

	for (int i = 0; i < 4; i++) {

		vector_t norm_ampl = amplitude_vecs[i] / amplitude_vecs[i].norm();

		mag_pl_detector::msg::MagneticPhasor3D mag_msg;
		mag_msg.header.frame_id = "drone";
		mag_msg.header.stamp = stamp;

		mag_msg.amplitudes.x = amplitude_vecs[i](0);
		mag_msg.amplitudes.y = amplitude_vecs[i](1);
		mag_msg.amplitudes.z = amplitude_vecs[i](2);

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

std::vector<vector_t> SineReconstructionPublisherNode::correctAmplitudes(std::vector<vector_t> amplitude_vecs) {

    vector_t D_vj_vecs[4];
    vector_t D_vj_norm_vecs[4];

    int misaligned_votes[] = {0,0,0,0};

    for (int i = 0; i < 4; i++) {

        vector_t D_vj = amplitude_vecs[i];

        if (i == 2) {
            D_vj = -D_vj;
        }

        D_vj_vecs[i] = D_vj;
        D_vj_norm_vecs[i] = D_vj / D_vj.norm();

        for (int j = 0; j < i; j++) {

            // cout << to_string(D_vj_norm_vecs[i].dot(D_vj_norm_vecs[j])) << "\n";

            if (D_vj_norm_vecs[i].dot(D_vj_norm_vecs[j]) < 0.5) {

                misaligned_votes[i]++;
                misaligned_votes[j]++;

            }
        }
    }

    int misaligned_index = -1;

    for (int i = 0; i < 4; i++) {

        if (misaligned_votes[i] > 2) {

            misaligned_index = i;
            break;

        }
    }

    if(misaligned_index > -1) {
        int compare_index;
        double smallest_dist = 9999999.;

        for (int i = 0; i < 4; i++) {
            if (i == misaligned_index){
                continue;
            }

            vector_t dist_vec = P_D_Mja_p_[misaligned_index] - P_D_Mja_p_[i];
            double dist = dist_vec.norm();
            if (dist < smallest_dist) {
                smallest_dist = dist;
                compare_index = i;
            }
        }

        // cout << "Found misaligned index: " << to_string(misaligned_index) << "\n";
        bool finished = false;

        float angles[] = {M_PI_2, M_PI, 3*M_PI_2};

        rotation_matrix_t best_R;
        double best_dot_prod = -9999;

        for (int i = 0; i < 3; i++) {

            for (int j = 0; j < 3; j++) {

                orientation_t eul(0,0,0);
                eul(i) = angles[j];
                rotation_matrix_t R = eulToR(eul);

                vector_t vec_rotated = R * D_vj_norm_vecs[misaligned_index];

                double dot_prod = vec_rotated.dot(D_vj_norm_vecs[compare_index]);
                if (dot_prod > best_dot_prod) {
                    best_dot_prod = dot_prod;
                    best_R = R;
                }
            }
        }

        D_vj_vecs[misaligned_index] = best_R * D_vj_vecs[misaligned_index];
    }

}


void SineReconstructionPublisherNode::projectVectors() {

    pl_dir_mutex_.lock(); {

        pl_dir_quat_hold_ = pl_dir_quat_;

    } pl_dir_mutex_.unlock();

    rotation_matrix_t pl_R = quatToMat(pl_dir_quat_hold_);
    vector_t unit_x(1,0,0);
    pl_unit_x_ = pl_R * unit_x;

    point_t origin(0,0,0);

    projection_plane_ = {
        .p = origin,
        .normal = pl_unit_x_
    };

    for (int i = 0; i < 4; i++) {

        vector_t D_Malpha_pi = (vector_t)projectPointOnPlane((point_t)v_drone_to_mags_[i], projection_plane_);
        v_proj_mags_[i] = D_Malpha_pi;

    }

    rotation_matrix_t D_P_R = quatToMat(pl_dir_quat_hold_);
	rotation_matrix_t D_P_R_T = D_P_R.transpose();

    for (int i = 0; i < 4; i++) {

        P_D_Mja_p_[i] = D_P_R_T * v_proj_mags_[i];

    }

}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<SineReconstructionPublisherNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}
