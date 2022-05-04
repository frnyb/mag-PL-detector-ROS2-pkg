/*********************************************************************************
 * Includes
 *********************************************************************************/

#include <cmath>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mag_pl_detector/msg/mag_measurements.hpp"
#include "mag_pl_detector/msg/sine_reconstruction.hpp"
#include "mag_pl_detector/msg/magnetic_phasors3_d.hpp"

#include "geometry.h"

/*********************************************************************************
 * Class
 *********************************************************************************/

class MagMeasurementsClass {
public:
	MagMeasurementsClass(mag_pl_detector::msg::MagMeasurements measurements_msg, rotation_matrix_t R_drone_to_mags[4],
		bool fixed_phase, bool invert_z, int max_n_samples);

	mag_pl_detector::msg::SineReconstruction GetMsg();
	mag_pl_detector::msg::MagneticPhasors3D GetPhasorsMsg(builtin_interfaces::msg::Time stamp);

private:
	const std::string mag_frame_names_[4] = {"mag0", "mag1", "mag2", "mag3"};

	Eigen::MatrixXd mag_samples_;
	Eigen::MatrixXd mag_times_;

	rotation_matrix_t R_drone_to_mags_[4];

	bool fixed_phase_;
	bool invert_z_;

	int max_n_samples_;

	int n_samples_;
	int phase_ref_idx_;

	float phases_[12];
	float amplitudes_[12];
	float offsets_[12];

	void loadData(mag_pl_detector::msg::MagMeasurements measurements_msg);
	void computePhaseRefChannel();
	void computeRemainingChannels();
	void computeAllChannels();

};