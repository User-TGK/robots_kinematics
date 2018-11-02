#include "Kinematics.h"

#include <iostream>

namespace robot_controller
{
std::optional<Matrix<double, 3, 4>> Kinematics::forward(
	Matrix<double, 6, 1> angles)
{
	for (std::size_t i = 0; i < angles.get_m(); ++i) {
		if (!std::isfinite(angles[i][0])) {
			return std::nullopt;
		}
	}

	if (!Kinematics::angles_within_limits(angles)) {
		return std::nullopt;
	}

	// Convert angles to radians.
	angles *= (M_PI / 180);

	/*
	 * keep names short because the formules below are unreadable with long
	 * variable names.
	 * p are the end positions of each link.
	 * a are the thetas in radians
	 * l are the joint lengths
	 */
	Matrix<double, 3, 4> p;
	const auto& a = angles;
	const auto& l = joint_lengths;

	p[0][0] = 0.0;
	p[1][0] = 0.0;
	p[2][0] = 0.0;

	p[0][1] = p[0][0] + (l[0][0] * std::sin(a[1][0]) * std::cos(a[0][0]));
	p[1][1] = p[1][0] + (l[0][0] * std::cos(a[1][0]));
	p[2][1] = p[2][0] + (l[0][0] * std::sin(a[1][0]) * std::sin(a[0][0]));

	p[0][2] = p[0][1]
		+ (l[1][0] * std::sin(a[1][0] + a[2][0]) * std::cos(a[0][0]));
	p[1][2] = p[1][1] + (l[1][0] * std::cos(a[1][0] + a[2][0]));
	p[2][2] = p[2][1]
		+ (l[1][0] * std::sin(a[1][0] + a[2][0]) * std::sin(a[0][0]));

	p[0][3] = p[0][2]
		+ (l[2][0] * std::sin(a[1][0] + a[2][0] + a[3][0])
			  * std::cos(a[0][0]));
	p[1][3] = p[1][2] + (l[2][0] * std::cos(a[1][0] + a[2][0] + a[3][0]));
	p[2][3] = p[2][2]
		+ (l[2][0] * std::sin(a[1][0] + a[2][0] + a[3][0])
			  * std::sin(a[0][0]));

	return p;
}

std::optional<Matrix<double, 6, 1>> Kinematics::inverse(
	const Matrix<double, 3, 1>& goal, const double gripper_angle,
	const double wrist_rotate)
{
	std::optional<Matrix<double, 6, 1>> solution;

	// Check if solution is in worksapce.
	static const double max_arm_length =
		joint_lengths[0][0] + joint_lengths[1][0] + joint_lengths[2][0];
	if (max_arm_length - goal.get_magnitude() <= 0) {
		return solution;
	}

	solution = Matrix<double, 6, 1>();

	const auto& l = joint_lengths;

	auto& s = solution.value();

	const auto gripper_rad = (M_PI * gripper_angle) / 180;
	const auto wrist_rotate_rad = (M_PI * wrist_rotate) / 180;

	double theta_0 = std::atan2(goal[2][0], goal[0][0]);
	double x = std::hypot(goal[2][0], goal[0][0])
		- l[2][0] * std::cos(gripper_rad);
	double y = goal[1][0] + l[2][0] * std::sin(gripper_rad);

	double cos_theta_2 =
		((x * x) + (y * y) - (l[0][0] * l[0][0]) - (l[1][0] * l[1][0]))
		/ (2 * l[0][0] * l[1][0]);

	s[2][0] = std::atan2(
		std::sqrt(1 - cos_theta_2 * cos_theta_2), cos_theta_2);

	s[1][0] = std::atan2(x, y)
		- std::atan2(l[1][0] * std::sin(s[2][0]),
			  l[0][0] + l[1][0] * std::cos(s[2][0]));

	s[3][0] = ((M_PI / 2) + gripper_rad) - s[1][0] - s[2][0];

	s[4][0] = 0;

	s[0][0] = theta_0;

	s[5][0] = -s[0][0] + wrist_rotate_rad;

	s *= (180.0 / M_PI);

	if (s[5][0] > 90) {
		s[5][0] = std::fmod(s[5][0], 90.0);
	} else if (s[5][0] < -90) {
		s[5][0] = -std::fmod(s[5][0], 90.0);
	}

	std::cout << "base: " << s[0][0] << "wrist: " << (wrist_rotate_rad * (180.0 / M_PI)) << ", result: " << s[5][0];
	std::cout << s << std::endl;

	const auto fk1 = forward(s);

	// std::cout << "goal:" << std::endl
	// 	  << goal << std::endl
	// 	  << "forwards: " << std::endl
	// 	  << (fk1.has_value() == true ? fk1.value().str() : "nullopt")
	// 	  << std::endl;

	if (!fk1.has_value()
		|| (fk1.value().template slice<0, 3, 3, 1>() - goal)
				.get_magnitude()
			> precision) {
		solution = std::nullopt;
	}

	return solution;
}

bool Kinematics::angles_within_limits(const Matrix<double, 6, 1>& angles)
{
	for (std::size_t i = 0; i < servo_limits.get_m(); ++i) {
		if (servo_limits[i][0] > angles[i][0]
			|| servo_limits[i][1] < angles[i][0]) {
			std::cout << "Servo " << i << " out of range, value "
				  << angles[i][0] << " is not between "
				  << servo_limits[i][0] << " and"
				  << servo_limits[i][1] << std::endl;
			return false;
		}
	}

	return true;
}

} // namespace robot_controller