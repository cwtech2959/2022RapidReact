// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/filter/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include "Conditioning.h"
#include <thread>
#include "Drivetrain.h"
#include "Logging.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <Debug.h>

class Robot : public frc::TimedRobot, public cwtech::Debug
{
public:
    Robot()
        : cwtech::Debug("Robot")
    {
    }
    void RobotInit() override
    {
        m_conditioning.SetDeadband(0.2);
        m_conditioning.SetExponent(4.0);
    }

    void AutonomousPeriodic() override
    {
        DriveWithJoystick(false);
        m_swerve.UpdateOdometry();
    }

    void TeleopPeriodic() override
    {
        DriveWithJoystick(false);
        m_swerve.UpdateOdometry();
        auto logLine = m_swerve.OutputOdometry();
        m_logging.Log(logLine);
    }

    void TeleopInit() override
    {
        m_logging.StartNewLogFile();
        if (!m_haveSetInitalPositions)
        {
            m_swerve.SetInitialSwervePositions();
            m_haveSetInitalPositions = true;
        }
        m_swerve.UpdateDashboardOnUpdate();
    }

    void TestPeriodic() override
    {
        auto drive = m_testDriveOrTurn.GetBoolean(true);
        auto motor = (int)m_testMotorCorner.GetNumber(1.0);
        auto percentage = m_testMotorPercentage.GetNumber(0.0);

        m_swerve.DirectMotorDrive(drive, motor, percentage);
    }

private:
    // frc::XboxController m_controller{0};
    frc::Joystick m_joystick{0};
    cwtech::UniformConditioning m_conditioning{};
    Drivetrain m_swerve{this};
    Logging m_logging;

    cwtech::DebugVariable m_debugX = Variable("Joystick/X", {0.0});
    cwtech::DebugVariable m_debugY = Variable("Joystick/Y", {0.0});
    cwtech::DebugVariable m_testDriveOrTurn = Variable("Test/Drive or Turn", {true});
    cwtech::DebugVariable m_testMotorCorner = Variable("Test/Test Motor Corner", {1.0});
    cwtech::DebugVariable m_testMotorPercentage = Variable("Test/Motor Percentage", {0.0});

    bool m_haveSetInitalPositions = false;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

    void DriveWithJoystick(bool fieldRelative)
    {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        auto x = m_joystick.GetX();
        x = -m_conditioning.Condition(x);
        auto xSpeed = x * Drivetrain::kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        auto y = m_joystick.GetY();
        y = m_conditioning.Condition(y);
        auto ySpeed = y * Drivetrain::kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        auto r = m_joystick.GetTwist();
        r = m_conditioning.Condition(r);
        auto rot = -r * Drivetrain::kMaxAngularSpeed;
        m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
    }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
