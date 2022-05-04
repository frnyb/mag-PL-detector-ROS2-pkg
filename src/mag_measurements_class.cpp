/*********************************************************************************
 * Includes
 *********************************************************************************/

#include "mag_measurements_class.h"

/*********************************************************************************
 * Methods
 *********************************************************************************/

MagMeasurementsClass::MagMeasurementsClass(mag_pl_detector::msg::MagMeasurements measurements_msg, rotation_matrix_t R_drone_to_mags[4],
	bool fixed_phase, bool invert_z, int max_n_samples) :
					mag_samples_(measurements_msg.count, 12), mag_times_(measurements_msg.count, 12) {

	fixed_phase_ = fixed_phase;
	invert_z_ = invert_z;

	max_n_samples_ = max_n_samples;

	for (int i = 0; i < 4; i++) {

		R_drone_to_mags_[i] = R_drone_to_mags[i];

	}

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

	if (n_samples_ > max_n_samples_) {

		n_samples_ = max_n_samples_;

	}

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

mag_pl_detector::msg::MagneticPhasors3D MagMeasurementsClass::GetPhasorsMsg(builtin_interfaces::msg::Time stamp) {

	std::vector<mag_pl_detector::msg::MagneticPhasor3D> phasors;

	for (int i = 0; i < 4; i++) {

		vector_t ampl(
			amplitudes_[i*3],
			amplitudes_[i*3+1],
			amplitudes_[i*3+2]
		);

		ampl = R_drone_to_mags_[i] * ampl;

		vector_t norm_ampl = ampl / ampl.norm();

		mag_pl_detector::msg::MagneticPhasor3D mag_msg;
		//mag_msg.header.frame_id = mag_frame_names_[i];
		mag_msg.header.frame_id = "drone";
		mag_msg.header.stamp = stamp;
		mag_msg.amplitudes.x = ampl(0);
		mag_msg.amplitudes.y = ampl(1);
		mag_msg.amplitudes.z = ampl(2);
		mag_msg.normalized_amplitudes.x = norm_ampl(0);
		mag_msg.normalized_amplitudes.y = norm_ampl(1);
		mag_msg.normalized_amplitudes.z = norm_ampl(2);
		mag_msg.phases.x = phases_[0];
		mag_msg.phases.y = phases_[1];
		mag_msg.phases.z = phases_[2];

		phasors.push_back(mag_msg);

	}

	mag_pl_detector::msg::MagneticPhasors3D msg;
	msg.phasors = phasors;

	return msg;

}