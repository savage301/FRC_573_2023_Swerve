// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/Timer.h>
#include <frc/controller/RamseteController.h>


#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
 void AutonomousInit() override{

   // Start the timer.
    m_timer.Start();

    // Send Field2d to SmartDashboard.
    frc::SmartDashboard::PutData(&m_field);

    // Reset the drivetrain's odometry to the starting pose of the trajectory.
    m_swerve.ResetOdometry(exampleTrajectory.InitialPose());

    // Send our generated trajectory to Field2d.
    m_field.GetObject("traj")->SetTrajectory(exampleTrajectory);

 }
  void AutonomousPeriodic() override {   
  // Update odometry.
    m_swerve.UpdateOdometry();

  // Update robot position on Field2d.
    m_field.SetRobotPose(m_swerve.GetPose());

  // Send Field2d to SmartDashboard.
    frc::SmartDashboard::PutData(&m_field);

  if (m_timer.Get() < exampleTrajectory.TotalTime()) {
  // Get the desired pose from the trajectory.
    auto desiredPose = exampleTrajectory.Sample(m_timer.Get());

  // Get the reference chassis speeds from the Ramsete Controller.
    auto refChassisSpeeds =
      m_ramseteController.Calculate(m_swerve.GetPose(), desiredPose);

  // Set the linear and angular speeds.
      m_swerve.Drive(refChassisSpeeds.vx, refChassisSpeeds.vy, refChassisSpeeds.omega,false);
    } 
    else {
      m_swerve.Drive(units::meters_per_second_t(0), units::meters_per_second_t(0), units::radians_per_second_t(0), false); 
    }


  }

  void TeleopPeriodic() override { DriveWithJoystick(false); }

 private:
  frc::XboxController m_controller{0};
  Drivetrain m_swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  // -------------- Added for Auto------------------------------
  frc::Trajectory exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(5_m, 5_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(6_m, 6_m), frc::Translation2d(7_m, 4_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(8_m, 5_m, frc::Rotation2d(0_deg)),
      // Pass the config
      m_swerve.auto_traj);

  // The Ramsete Controller to follow the trajectory.
  frc::RamseteController m_ramseteController;

  // The timer to use during the autonomous period.
  frc::Timer m_timer;

  // Create Field2d for robot and trajectory visualizations.
  frc::Field2d m_field;

// -------------------------------------------------------------

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_controller.GetLeftY(), 0.02)) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_controller.GetLeftX(), 0.02)) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_controller.GetRightX(), 0.02)) *
                     Drivetrain::kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);

   frc::SmartDashboard::PutString("xSpeed", std::to_string(xSpeed.value()));
   frc::SmartDashboard::PutString("ySpeed", std::to_string(ySpeed.value()));
   frc::SmartDashboard::PutString("rot", std::to_string(rot.value()));
  m_swerve.UpdateOdometry();
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
