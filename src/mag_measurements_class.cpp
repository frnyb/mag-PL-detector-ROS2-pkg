/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "mag_measurements_class.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

MagMeasurementsClass::MagMeasurementsClass(mag_pl_detector::msg::MagMeasurements measurements_msg, bool fixed_phase, bool invert_z) :
					mag_samples_(measurements_msg.count, 12), mag_times_(measurements_msg.count, 12) {

	fixed_phase_ = fixed_phase;
	invert_z_ = invert_z;

	loadData(measurements_msg);

	if (fixed_phase) {

		computePhaseRefChannel();
		computeRemainingChannels();

	} else {

		computeAllChannels();

	}

}

void MagMeasurementsClass::loadData(mag_pl_detector::msg::MagMeasurements measurements_msg) {

	n_samples_ = measurements_msg.count;

	Eigen::VectorXd max_vals(12, 1);
	Eigen::VectorXd min_vals(12, 1);

	for(int i = 0; i < n_samples_; i++) {

		for (int j = 0; j < 12; j++) {

			mag_samples_(i,j) = measurements_msg.samples[i].data[j];

			if (i == 0) {

				max_vals(j,0) = mag_samples_(i,j);
				min_vals(j,0) = mag_samples_(i,j);

			} else if (mag_samples_(i,j) > max_vals(j,0)) {

				max_vals(j,0) = mag_samples_(i,j);

			} else if (mag_samples_(i,j) < min_vals(j,0)) {

				min_vals(j,0) = mag_samples_(i,j);

			}

			if (j == 0) {

				if (i == 0) {

					mag_times_(i,j) = 0;

				} else {

					mag_times_(i,j) = mag_times_(i-1,j) + measurements_msg.samples[i].time_offset[j] * 0.00000001;

				}

			} else {

				mag_times_(i,j) = mag_times_(i,0) + measurements_msg.samples[i].time_offset[j] * 0.00000001;

			}
		}
	}

	Eigen::VectorXd min_max_diff = max_vals - min_vals;

	phase_ref_idx_ = 0;

	for (int i = 1; i < 12; i++) {

		if (min_max_diff(i,0) > min_max_diff(phase_ref_idx_, 0)) {

			phase_ref_idx_ = i;

		}
	}
}

void MagMeasurementsClass::computePhaseRefChannel() {

	Eigen::MatrixXd A(n_samples_, 3);
	Eigen::VectorXd b(n_samples_);

	for(int i = 0; i < n_samples_; i++) {

		A(i,0) = 1;
		A(i,1) = sin(2*M_PI*50*mag_times_(i, phase_ref_idx_));
		A(i,2) = cos(2*M_PI*50*mag_times_(i, phase_ref_idx_));

		b(i) = mag_samples_(i,phase_ref_idx_);

	}

	Eigen::Vector3d x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	offsets_[phase_ref_idx_] = x(0);
	amplitudes_[phase_ref_idx_] = sqrt(x(1)*x(1) + x(2)*x(2));

	float phase = atan2(x(2), x(1));

	for (int i = 0; i < 12; i++) {
		phases_[i] = phase;
	}

}

void MagMeasurementsClass::computeRemainingChannels() {

	Eigen::MatrixXd A(n_samples_*11, 2*11);
	Eigen::VectorXd b(n_samples_*11);

	for (int i = 0; i < 11; i++) {

		int ch_idx = i;

		if (i >= phase_ref_idx_) {

			ch_idx++;

		}

		for (int j = 0; j < n_samples_; j++) {

			int idx = i*n_samples_ + j;

			A(idx, i) = 1;
			A(idx, 11+i) = sin(2*M_PI*50*mag_times_(j,ch_idx) + phases_[0]);

			b(idx) = mag_samples_(j,ch_idx);

		}
	}

	Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	for (int i = 0; i < 11; i++) {

		int ch_idx = i;

		if (i >= phase_ref_idx_) {

			ch_idx++;

		}

		offsets_[ch_idx] = x(i);
		amplitudes_[ch_idx] = x(11+i);

	}
}

void MagMeasurementsClass::computeAllChannels() {

	Eigen::MatrixXd A(n_samples_*12, 3*12);
	Eigen::VectorXd b(n_samples_*12);

	for (int i = 0; i < 12; i++) {

		for (int j = 0; j < n_samples_; j++) {

			int idx = i*n_samples_ + j;

			A(idx, i) = 1;
			A(idx, 12+i) = sin(2*M_PI*50*mag_times_(j, i));
			A(idx, 24+i) = cos(2*M_PI*50*mag_times_(j, i));

			b(idx) = mag_samples_(j, i);

		}
	}

	Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	for (int i = 0; i < 12; i++) {

		offsets_[i] = x(i);
		amplitudes_[i] = sqrt(x(12+i)*x(12+i) + x(24+i)*x(24+i));
		phases_[i] = atan2(x(24+i), x(12+i));

	}
}

mag_pl_detector::msg::SineReconstruction MagMeasurementsClass::GetMsg() {

	mag_pl_detector::msg::SineReconstruction msg;

	int ampl_n = sizeof(amplitudes_) / sizeof(amplitudes_[0]);
	std::vector<float> ampltude_vec(amplitudes_, amplitudes_+ampl_n);

	int offsets_n = sizeof(offsets_) / sizeof(offsets_[0]);
	std::vector<float> offsets_vec(offsets_, offsets_+offsets_n);

	int phases_n = sizeof(phases_) / sizeof(phases_[0]);
	std::vector<float> phases_vec(phases_, phases_+phases_n);

	msg.amplitudes = ampltude_vec;
	msg.offsets = offsets_vec;
	msg.phases = phases_vec;

	return msg;
	
}

std::vector<geometry_msgs::msg::Vector3Stamped> MagMeasurementsClass::GetAmplitudeVectorMsgs(builtin_interfaces::msg::Time stamp) {

	geometry_msgs::msg::Vector3Stamped mag0_msg;
	mag0_msg.header.frame_id = "mag0";
	mag0_msg.header.stamp = stamp;
	mag0_msg.vector.x = amplitudes_[0];
	mag0_msg.vector.y = amplitudes_[1];
	mag0_msg.vector.z = amplitudes_[2];

	geometry_msgs::msg::Vector3Stamped mag1_msg;
	mag1_msg.header.frame_id = "mag1";
	mag1_msg.header.stamp = stamp;
	mag1_msg.vector.x = amplitudes_[3];
	mag1_msg.vector.y = amplitudes_[4];
	mag1_msg.vector.z = amplitudes_[5];

	geometry_msgs::msg::Vector3Stamped mag2_msg;
	mag2_msg.header.frame_id = "mag2";
	mag2_msg.header.stamp = stamp;
	mag2_msg.vector.x = amplitudes_[6];
	mag2_msg.vector.y = amplitudes_[7];
	mag2_msg.vector.z = amplitudes_[8];

	geometry_msgs::msg::Vector3Stamped mag3_msg;
	mag3_msg.header.frame_id = "mag3";
	mag3_msg.header.stamp = stamp;
	mag3_msg.vector.x = amplitudes_[9];
	mag3_msg.vector.y = amplitudes_[10];
	mag3_msg.vector.z = amplitudes_[11];

	if (invert_z_) {
		mag0_msg.vector.z = - mag0_msg.vector.z;
		mag1_msg.vector.z = - mag1_msg.vector.z;
		mag2_msg.vector.z = - mag2_msg.vector.z;
		mag3_msg.vector.z = - mag3_msg.vector.z;
	}
	

	std::vector<geometry_msgs::msg::Vector3Stamped> vector;
	vector.push_back(mag0_msg);
	vector.push_back(mag1_msg);
	vector.push_back(mag2_msg);
	vector.push_back(mag3_msg);

	return vector;

}

std::vector<mag_pl_detector::msg::MagneticPhasor> MagMeasurementsClass::GetPhasorMsgs(builtin_interfaces::msg::Time stamp) {

	mag_pl_detector::msg::MagneticPhasor mag0_msg;
	mag0_msg.header.frame_id = "mag0";
	mag0_msg.header.stamp = stamp;
	mag0_msg.amplitudes.x = amplitudes_[0];
	mag0_msg.amplitudes.y = amplitudes_[1];
	mag0_msg.amplitudes.z = amplitudes_[2];
	mag0_msg.phases.x = phases_[0];
	mag0_msg.phases.y = phases_[1];
	mag0_msg.phases.z = phases_[2];

	mag_pl_detector::msg::MagneticPhasor mag1_msg;
	mag1_msg.header.frame_id = "mag1";
	mag1_msg.header.stamp = stamp;
	mag1_msg.amplitudes.x = amplitudes_[3];
	mag1_msg.amplitudes.y = amplitudes_[4];
	mag1_msg.amplitudes.z = amplitudes_[5];
	mag1_msg.phases.x = phases_[3];
	mag1_msg.phases.y = phases_[4];
	mag1_msg.phases.z = phases_[5];

	mag_pl_detector::msg::MagneticPhasor mag2_msg;
	mag2_msg.header.frame_id = "mag2";
	mag2_msg.header.stamp = stamp;
	mag2_msg.amplitudes.x = amplitudes_[6];
	mag2_msg.amplitudes.y = amplitudes_[7];
	mag2_msg.amplitudes.z = amplitudes_[8];
	mag2_msg.phases.x = phases_[6];
	mag2_msg.phases.y = phases_[7];
	mag2_msg.phases.z = phases_[8];

	mag_pl_detector::msg::MagneticPhasor mag3_msg;
	mag3_msg.header.frame_id = "mag3";
	mag3_msg.header.stamp = stamp;
	mag3_msg.amplitudes.x = amplitudes_[9];
	mag3_msg.amplitudes.y = amplitudes_[10];
	mag3_msg.amplitudes.z = amplitudes_[11];
	mag3_msg.phases.x = phases_[9];
	mag3_msg.phases.y = phases_[10];
	mag3_msg.phases.z = phases_[11];

	std::vector<mag_pl_detector::msg::MagneticPhasor> vector;
	vector.push_back(mag0_msg);
	vector.push_back(mag1_msg);
	vector.push_back(mag2_msg);
	vector.push_back(mag3_msg);

	return vector;

}