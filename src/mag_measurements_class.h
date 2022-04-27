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
#include "mag_pl_detector/msg/magnetic_phasor.hpp"

/*********************************************************************************
 * Class
 *********************************************************************************/

class MagMeasurementsClass {
public:
	MagMeasurementsClass(mag_pl_detector::msg::MagMeasurements measurements_msg, bool fixed_phase);

	mag_pl_detector::msg::SineReconstruction GetMsg();
	std::vector<geometry_msgs::msg::Vector3Stamped> GetAmplitudeVectorMsgs(builtin_interfaces::msg::Time stamp);
	std::vector<mag_pl_detector::msg::MagneticPhasor> GetPhasorMsgs(builtin_interfaces::msg::Time stamp);

private:
	Eigen::MatrixXd mag_samples_;
	Eigen::MatrixXd mag_times_;

	bool fixed_phase_;

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