#ifndef ROBOT_CONTROLLER_KINEMATICS_HPP
#define ROBOT_CONTROLLER_KINEMATICS_HPP

#include "Matrix.h"

#include <cmath>
#include <optional>
#include <utility>

namespace robot_controller
{
/**
 * \brief Contains the functions to compute the forward and inverse kinematics
 * for the Lyxnmotion AL5D robot arm.
 */
class Kinematics
{
public:
	/**
	 * \brief Object is not constructable so default constructor is deleted.
	 */
	Kinematics() = delete;

	/**
	 * \brief Default destructor
	 */
	~Kinematics() = default;

	/**
	 * \brief Object is not constructable so copy constructor is deleted.
	 *
	 * \param k
	 */
	Kinematics(const Kinematics& k) = delete;

	/**
	 * \brief Object is not constructable so move constructor is deleted.
	 *
	 * \param k
	 */
	Kinematics(Kinematics&& k) = delete;

	/**
	 * \brief Object is not constructable so copy assignment is deleted.
	 *
	 * \param k
	 * \return
	 */
	Kinematics& operator=(const Kinematics& k) = delete;

	/**
	 * \brief Object is not constructable so move assignment is deleted.
	 *
	 * \param k
	 * \return
	 */
	Kinematics& operator=(Kinematics&& k) = delete;

	/**
	 * \brief Calculate forward kinematics given the angles of the robot arm
	 *
	 * \param angles The angles of each joint.
	 * \return The position of each joint starting from base. Where it's in
	 * the format x, y, z. So the last row in the matrix contains the end
	 * effector position. If it can't be calculated due to servo limitations
	 * the optional is empty.
	 */
	static std::optional<Matrix<double, 3, 4>> forward(
		Matrix<double, 6, 1> angles);

	/**
	 * \brief Calculate joint angles based on the position the end effector
	 * of the robot arm should have.
	 *
	 * \param goal The goal to reach.
	 * \return Matrix containing the result if found
	 */

	/**
	 * \brief Calculate joint angles based on the position the end effector
	 * of the robot arm should have.
	 *
	 * \param goal The goal to reach.
	 * \param gripper_angle The angle which the gripper has to stand. Where 0 degree is parallel to the ground
	 * \param wrist_rotate The rotation of the gripper
	 * \return Matrix containing the result if found
	 */
	static std::optional<Matrix<double, 6, 1>> inverse(
		const Matrix<double, 3, 1>& goal, const double gripper_angle,
		const double wrist_rotate);

private:
	/**
	 * \brief Check if angles given for each servo are within their hardware
	 * limits.
	 *
	 * \param angles The angles of each servo to test.
	 * \return true If within limits.
	 * \return false If outside of limits or optional is empty.
	 */
	static bool angles_within_limits(const Matrix<double, 6, 1>& angles);

	/**
	 * \brief The precision in meter that counts as being close enough to
	 * the goal to match.
	 */
	static constexpr double precision = 0.001;

	/**
	 * \brief The length of the links in meter.
	 */
	static constexpr Matrix<double, 3, 1> joint_lengths = {
		{0.14605}, {0.187325}, {0.09}};

	/**
	 * \brief Contains the limits for all joints
	 */
	static constexpr Matrix<double, 6, 2> servo_limits = {{-90.0, 90.0},
		{-45.0, 90.0}, {0, 145.0}, {-90.0, 90.0}, {0, 31}, {-90, 90.0}};
};
} // namespace robot_controller

#endif