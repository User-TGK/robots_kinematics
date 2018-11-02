#include <robot_arm_interface/RobotArm.h>

#include <ssc32u/Commands.h>

#include <iostream>

namespace ArmControl
{
/// \brief Map an input variable in a range to a output corresponding to the
/// same relative position. \param inStart The start of the current range \param
/// inEnd The end of the current range. \param outStart The start of the output
/// range. \param outEnd The end of the output range. \param value The value to
/// map. \return The mapped value.
double map(double inStart, double inEnd, double outStart, double outEnd,
	double value)
{
	return (value - inStart) / (inEnd - inStart) * (outEnd - outStart)
		+ outStart;
}

/// \brief Translate degree to a PWM value.
/// \param degree The degree to translate.
/// \return The pulse width to send.
std::uint16_t DegreesToPWM(const JointPositionSetting& setting, double degree)
{
	return static_cast<std::uint16_t>(map(setting.minDegree,
		setting.maxDegree, setting.minPWM, setting.maxPWM, degree));
}

/// \brief Translate PWM to a degree value.
/// \param PWM The PWM to translate.
/// \return The degrees that correspond to the pwm to send.
double PWMToDegrees(const JointPositionSetting& setting, std::uint16_t PWM)
{
	return map(setting.minPWM, setting.maxPWM, setting.minDegree,
		setting.maxDegree, PWM);
}

/// \brief Translate millimeters to a PWM value.
/// \param millimeters The millimeters to translate.
/// \return The pulse width to send.
std::uint16_t MmToPWM(const GripperPositionSetting& setting, double millimeters)
{
	return static_cast<std::uint16_t>(map(setting.minMm, setting.maxMm,
		setting.minPWM, setting.maxPWM, millimeters));
}

/// \brief Translate PWM to a millimeter value for the gripper
/// \param PWM The PWM to translate.
/// \return The millimeters that correspond to the pwm to send.
double PWMToMm(const GripperPositionSetting& setting, std::uint16_t PWM)
{
	return map(setting.minPWM, setting.maxPWM, setting.minMm, setting.maxMm,
		PWM);
}

/// \brief Translate degrees per second to microsecond per second.
/// \param degreePerSecond The amount of degrees to move per second/
/// \return The microseconds per second value that maps to that speed.
std::uint32_t DegreesPerSecToMicrosecondPerSecond(double degreePerSecond)
{
	return static_cast<std::uint32_t>((degreePerSecond / 90) * 1000);
}

/// \brief Create a servo move command from a joint position setting and a joint
/// position state to reach. \param setting The settings, includes channel info
/// and min and max offsets supported. \param position The position to reach.
/// \param timeToReachPosition The maximum time to reach that position.
/// \return The created servo move command.
ssc32drv::MoveCmd DestinationToMoveCmd(const JointPositionSetting& setting,
	const JointPosition& position, uint16_t timeToReachPosition)
{
	if (position.degree > setting.maxDegree
		|| position.degree < setting.minDegree) {
		throw std::out_of_range(
			"Desired joint position is out of available range.");
	}

	auto pwm = DegreesToPWM(setting, position.degree);
	auto speed = DegreesPerSecToMicrosecondPerSecond(
		position.maxDegreesPerSecond);

	return ssc32drv::MoveCmd(
		setting.channel, pwm, timeToReachPosition, speed);
}

/// \brief Create a servo move command from a joint position setting and a joint
/// position state to reach. \param setting The settings, includes channel info
/// and min and max offsets supported. \param position The position to reach.
/// \param timeToReachPosition The maximum time to reach that position.
/// \return The created servo move command.
ssc32drv::MoveCmd DestinationToMoveCmd(const GripperPositionSetting& setting,
	const GripperPosition& position, uint16_t timeToReachPosition)
{
	if (position.distance > setting.maxMm
		|| position.distance < setting.minMm) {
		throw std::out_of_range(
			"Desired joint position is out of available range.");
	}

	auto pwm = MmToPWM(setting, position.distance);
	auto speed =
		DegreesPerSecToMicrosecondPerSecond(position.maxMmPerSecond);

	return ssc32drv::MoveCmd(
		setting.channel, pwm, timeToReachPosition, speed);
}

const JointPositions& RobotArm::GetParkPosition()
{
	static const JointPositions pos = {
		{-90, 0}, {-30, 0}, {135, 0}, {70, 0}, {0, 0}, {0, 0}};

	return pos;
}

const JointPositions& RobotArm::GetReadyPosition()
{
	static const JointPositions pos = {
		{-90, 0}, {-30, 0}, {120, 0}, {0, 0}, {31, 0}, {0, 0}};

	return pos;
}

const JointPositions& RobotArm::GetStraightUpPosition()
{
	static const JointPositions pos = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {31, 0}, {0, 0}};

	return pos;
}

const JointPositions& RobotArm::GetMaxDegreesPerSecond()
{
	static const JointPositions pos = {
		{0, 300}, {0, 180}, {0, 180}, {0, 200}, {0, 180}, {0, 150}};

	return pos;
}

RobotArm::RobotArm(const std::string& robotArmConfigJson,
	const std::string& driverConfigJson) :
	config(robotArmConfigJson),
	driver(driverConfigJson)
{
}

JointPositions RobotArm::GetCurrentJointPositions() const
{
	JointPositions positions;

	positions.base.degree = PWMToDegrees(this->config.base,
		this->driver.GetFeedback(this->config.base.channel));
	positions.shoulder.degree = PWMToDegrees(this->config.shoulder,
		this->driver.GetFeedback(this->config.shoulder.channel));
	positions.elbow.degree = PWMToDegrees(this->config.elbow,
		this->driver.GetFeedback(this->config.elbow.channel));
	positions.wrist.degree = PWMToDegrees(this->config.wrist,
		this->driver.GetFeedback(this->config.wrist.channel));
	positions.gripper.distance = PWMToMm(this->config.gripper,
		this->driver.GetFeedback(this->config.gripper.channel));
	positions.wristRotate.degree = PWMToDegrees(this->config.wristRotate,
		this->driver.GetFeedback(this->config.wristRotate.channel));

	return positions;
}

bool RobotArm::IsMoving() const
{
	return this->driver.LastMoveComplete() == false;
}

void RobotArm::Move(const RobotArmDestination& destination)
{
	const double baseUsPerDegree = 1 / 0.102;
	const double shoulderUsPerDegree = 1 / 0.105;
	const double elbowUsPerDegree = 1 / 0.109;
	const double gripperUsPerDegree = 1 / 0.100;

	const double centerbase = 1500 -2.5 * baseUsPerDegree;
	const double centerShoulder = 1500 - 1.5 * shoulderUsPerDegree;
	const double centerElbow = 1500 + 5.5 * elbowUsPerDegree;
	const double centerGripper = 1525;

	const double baseCenter = 0;
	const double shoulderCenter = 0;
	const double elbowCenter = 90;
	const double gripperCenter = 0;

	double pwma = centerbase
		- ((destination.positions.base.degree - baseCenter)
			  * baseUsPerDegree);
	double pwmb = centerShoulder
		- ((destination.positions.shoulder.degree - shoulderCenter)
			  * shoulderUsPerDegree);
	double pwmc = centerElbow
		+ ((destination.positions.elbow.degree - elbowCenter)
			  * elbowUsPerDegree);
	double pwmd = centerGripper
		- ((destination.positions.wrist.degree - gripperCenter)
			  * gripperUsPerDegree);

	const auto base = ssc32drv::MoveCmd(0, (std::uint16_t)pwma,
		destination.timeToReachDestination,
		DegreesPerSecToMicrosecondPerSecond(
			destination.positions.wrist.maxDegreesPerSecond));

	const auto shoulder = ssc32drv::MoveCmd(1, (std::uint16_t)pwmb,
		destination.timeToReachDestination,
		DegreesPerSecToMicrosecondPerSecond(
			destination.positions.shoulder.maxDegreesPerSecond));

	const auto elbow = ssc32drv::MoveCmd(2, (std::uint16_t)pwmc,
		destination.timeToReachDestination,
		DegreesPerSecToMicrosecondPerSecond(
			destination.positions.elbow.maxDegreesPerSecond));

	const auto wrist = ssc32drv::MoveCmd(3, (std::uint16_t)pwmd,
		destination.timeToReachDestination,
		DegreesPerSecToMicrosecondPerSecond(
			destination.positions.wrist.maxDegreesPerSecond));

	auto commands = std::vector<ssc32drv::MoveCmd>{
		base,
		shoulder,
		elbow,
		wrist,
		DestinationToMoveCmd(this->config.gripper,
			destination.positions.gripper,
			destination.timeToReachDestination),
		DestinationToMoveCmd(this->config.wristRotate,
			destination.positions.wristRotate,
			destination.timeToReachDestination),
	};

	auto multimove = ssc32drv::MultiMoveCmd(commands);

	this->driver.MoveServo(multimove);
}

void RobotArm::Stop()
{
	this->driver.StopAllServos();
}

bool RobotArm::ExceedsMaxSpeed(const RobotArmDestination& destination)
{
	const auto& maxDegreesPerSecond = GetMaxDegreesPerSecond();
	const auto currentPos = this->GetCurrentJointPositions();

	if (destination.positions.base.maxDegreesPerSecond
		> maxDegreesPerSecond.base.maxDegreesPerSecond) {
		return true;
	} else if (destination.positions.shoulder.maxDegreesPerSecond
		> maxDegreesPerSecond.shoulder.maxDegreesPerSecond) {
		return true;
	} else if (destination.positions.elbow.maxDegreesPerSecond
		> maxDegreesPerSecond.elbow.maxDegreesPerSecond) {
		return true;
	} else if (destination.positions.wrist.maxDegreesPerSecond
		> maxDegreesPerSecond.wrist.maxDegreesPerSecond) {
		return true;
	} else if (destination.positions.gripper.maxMmPerSecond
		> maxDegreesPerSecond.gripper.maxMmPerSecond) {
		return true;
	} else if (destination.positions.wristRotate.maxDegreesPerSecond
		> maxDegreesPerSecond.wristRotate.maxDegreesPerSecond) {
		return true;
	}

	if (std::abs(destination.positions.base.degree - currentPos.base.degree)
			/ destination.timeToReachDestination
		> (maxDegreesPerSecond.base.maxDegreesPerSecond / 1000)) {
		return true;
	} else if (std::abs(destination.positions.shoulder.degree
			   - currentPos.shoulder.degree)
			/ destination.timeToReachDestination
		> (maxDegreesPerSecond.shoulder.maxDegreesPerSecond / 1000)) {
		return true;
	} else if (std::abs(destination.positions.elbow.degree
			   - currentPos.elbow.degree)
			/ destination.timeToReachDestination
		> (maxDegreesPerSecond.elbow.maxDegreesPerSecond / 1000)) {
		return true;
	} else if (std::abs(destination.positions.wrist.degree
			   - currentPos.wrist.degree)
			/ destination.timeToReachDestination
		> (maxDegreesPerSecond.wrist.maxDegreesPerSecond / 1000)) {
		return true;
	} else if (std::abs(destination.positions.gripper.distance
			   - currentPos.gripper.distance)
			/ destination.timeToReachDestination
		> (maxDegreesPerSecond.gripper.maxMmPerSecond / 1000)) {
		return true;
	} else if (std::abs(destination.positions.wristRotate.degree
			   - currentPos.wristRotate.degree)
			/ destination.timeToReachDestination
		> (maxDegreesPerSecond.wristRotate.maxDegreesPerSecond
			  / 1000)) {
		return true;
	}

	return false;
}
}