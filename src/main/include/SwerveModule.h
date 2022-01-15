// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <frc/DutyCycle.h>
#include <frc/DigitalInput.h>
#include <Debug.h>

class SwerveModule : public cwtech::Debug
{
public:
    SwerveModule(int driveMotorChannel, int turningMotorChannel, int dutyCycleChannel, std::string name, units::radian_t turnOffset, Debug* parent = nullptr);
    frc::SwerveModuleState GetState();
    void SetDesiredState(const frc::SwerveModuleState &state);
    void SetInitialPosition();
    void DirectDrive(bool drive, double percentage);
    void UpdateDashboardOnEnable();
    double GetAbsoluteEncoderPosition();

private:
    static constexpr double kWheelRadius = 2.0 * 0.254; // 2" * 0.254 m / inch
    static constexpr int kEncoderResolution = 4096;
    const double kGearboxRatio = 1.0 / 6.86; // One turn of the wheel is 6.86 turns of the motor
    const double kDrivePositionFactor = (2.0 * wpi::numbers::pi * kWheelRadius * kGearboxRatio);

    static constexpr auto kModuleMaxAngularVelocity =
        wpi::numbers::pi * 1_rad_per_s; // radians per second
    static constexpr auto kModuleMaxAngularAcceleration =
        wpi::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

    std::string m_name;

    rev::CANSparkMax m_driveMotor;
    rev::CANSparkMax m_turningMotor;

    frc::DigitalInput m_dutyCycleInput;
    frc::DutyCycle m_dutyCycleEncoder;

    rev::SparkMaxRelativeEncoder m_driveEncoder{m_driveMotor.GetEncoder()};
    rev::SparkMaxAlternateEncoder m_turningEncoder{m_turningMotor.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::Type::kQuadrature, kEncoderResolution)};

    rev::SparkMaxPIDController m_drivePIDController;
    rev::SparkMaxPIDController m_turningPIDController;

    units::radian_t m_turnOffset;

    cwtech::DebugVariable m_currentAngle = Variable("Current Angle", 0.0);
    cwtech::DebugVariable m_targetAngle = Variable("Target Angle", 0.0);
    cwtech::DebugVariable m_currentSpeed = Variable("Current Speed", 0.0);
    cwtech::DebugVariable m_targetSpeed = Variable("Target Speed", 0.0);
    cwtech::DebugVariable m_currentAngleAbs = Variable("Current Angle Abs", 0.0);
    cwtech::DebugVariable m_currentAngleAbsFreq = Variable("Current Angle Abs (Freq)", 0.0);
    cwtech::DebugVariable m_P = Variable("P", 0.0);
    cwtech::DebugVariable m_I = Variable("I", 0.0);
    cwtech::DebugVariable m_D = Variable("D", 0.0);
    cwtech::DebugVariable m_FF = Variable("FF", 0.2);
};
