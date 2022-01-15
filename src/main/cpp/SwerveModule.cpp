// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
						   const int turningMotorChannel,
						   const int dutyCycleChannel,
						   std::string name,
						   units::radian_t turnOffset,
						   Debug* parent)
	: cwtech::Debug("Swerve Module/" + name, parent), m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
	  m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
	  m_dutyCycleInput(dutyCycleChannel),
	  m_dutyCycleEncoder(m_dutyCycleInput),
	  m_drivePIDController(m_driveMotor.GetPIDController()),
	  m_turningPIDController(m_turningMotor.GetPIDController()),
	  m_turnOffset(turnOffset)

{
	// Set the distance per pulse for the drive encoder. We can simply use the
	// distance traveled for one rotation of the wheel divided by the encoder
	// resolution.
	m_driveEncoder.SetPositionConversionFactor(kDrivePositionFactor);
	m_driveEncoder.SetVelocityConversionFactor(kDrivePositionFactor / 60.0);

	m_drivePIDController.SetP(0.0003);
	m_drivePIDController.SetI(0.0);
	m_drivePIDController.SetD(0.0);
	m_drivePIDController.SetFF(0.0002);
	m_drivePIDController.SetIZone(600);

	m_turningPIDController.SetFeedbackDevice(m_turningEncoder);
	m_turningPIDController.SetP(0.4);
	m_turningPIDController.SetI(0.00001);
	m_turningPIDController.SetD(0.0);
	m_turningPIDController.SetIZone(1.0);

	// Set the distance (in this case, angle) per pulse for the turning encoder.
	// This is the the angle through an entire rotation (2 * wpi::numbers::pi)
	// divided by the encoder resolution.
	m_turningEncoder.SetPositionConversionFactor(2.0 * wpi::numbers::pi /*/ kEncoderResolution*/);
	m_name = name;
}

double SwerveModule::GetAbsoluteEncoderPosition()
{
	auto initalPosition = m_dutyCycleEncoder.GetOutput();								   // 0 to 1
	auto initalPositionInRadians = units::radian_t(initalPosition * 2 * wpi::numbers::pi); // radians
	auto initalPositionInRadiansScaled = frc::Rotation2d(initalPositionInRadians - m_turnOffset).Radians();
	return initalPositionInRadiansScaled.to<double>();
}

frc::SwerveModuleState SwerveModule::GetState()
{
	return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
			frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()))};
}

void SwerveModule::SetDesiredState(
	const frc::SwerveModuleState &referenceState)
{
	// Optimize the reference state to avoid spinning further than 90 degrees
	const auto state = frc::SwerveModuleState::Optimize(
		referenceState, units::radian_t(m_turningEncoder.GetPosition()));

	m_currentSpeed.PutNumber(m_driveEncoder.GetVelocity());
	m_targetSpeed.PutNumber(state.speed.to<double>());
	m_drivePIDController.SetReference(state.speed.to<double>() / kDrivePositionFactor, rev::CANSparkMax::ControlType::kVelocity);

	auto delta = state.angle - frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()));
	auto setpoint = units::radian_t(m_turningEncoder.GetPosition()) + delta.Radians();

	m_currentAngle.PutNumber(frc::Rotation2d{units::radian_t{m_turningEncoder.GetPosition()}}.Radians().to<double>());
	m_targetAngle.PutNumber(setpoint.to<double>());
	m_turningPIDController.SetReference(setpoint.to<double>(), rev::CANSparkMax::ControlType::kPosition);

	m_currentAngleAbs.PutNumber(GetAbsoluteEncoderPosition());
	m_currentAngleAbsFreq.PutNumber(m_dutyCycleEncoder.GetFrequency());
}

void SwerveModule::SetInitialPosition()
{
	m_turningEncoder.SetPosition(GetAbsoluteEncoderPosition() /* / m_turningEncoder.GetPositionConversionFactor()*/);
	m_currentAngleAbs.PutNumber(GetAbsoluteEncoderPosition());
	m_currentAngle.PutNumber(frc::Rotation2d{units::radian_t{m_turningEncoder.GetPosition()}}.Radians().to<double>());

	auto p = m_P.GetNumber(m_drivePIDController.GetP());
	auto i = m_I.GetNumber(m_drivePIDController.GetI());
	auto d = m_D.GetNumber(m_drivePIDController.GetD());
	auto ff = m_FF.GetNumber(m_drivePIDController.GetFF());

	m_drivePIDController.SetP(p);
	m_drivePIDController.SetI(i);
	m_drivePIDController.SetD(d);
	m_drivePIDController.SetFF(ff);
}

void SwerveModule::DirectDrive(bool drive, double percentage)
{
	if (drive)
		m_driveMotor.Set(percentage);
	else
		m_turningMotor.Set(percentage);
}

void SwerveModule::UpdateDashboardOnEnable()
{
	auto p = m_P.GetNumber(m_drivePIDController.GetP());
	auto i = m_I.GetNumber(m_drivePIDController.GetI());
	auto d = m_D.GetNumber(m_drivePIDController.GetD());

	m_drivePIDController.SetP(p);
	m_drivePIDController.SetI(i);
	m_drivePIDController.SetD(d);
}