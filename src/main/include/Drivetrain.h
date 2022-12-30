// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/trajectory/TrajectoryConfig.h>

#include <wpi/numbers>

#include "SwerveModule.h"

#include "frc/ADXRS450_Gyro.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d& pose);
  frc::Pose2d GetPose() const;

  static constexpr units::meters_per_second_t kMaxSpeed =
      5.0_mps;  // 5 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::numbers::pi};  // 1/2 rotation per second

  static constexpr auto kMaxAcceleration =
     units::meters_per_second_squared_t(2.5);  // meters per second^2
frc::TrajectoryConfig auto_traj {kMaxSpeed,kMaxAcceleration};

 private:
  frc::Translation2d m_frontLeftLocation{+0.305_m, +0.305_m};
  frc::Translation2d m_frontRightLocation{+0.305_m, -0.305_m};
  frc::Translation2d m_backLeftLocation{-0.305_m, +0.305_m};
  frc::Translation2d m_backRightLocation{-0.305_m, -0.305_m};
  
  SwerveModule m_frontLeft{1, 2, 3};
  SwerveModule m_frontRight{4, 5, 6};
  SwerveModule m_backLeft{7, 8, 9};
  SwerveModule m_backRight{10, 11, 12};

  frc::ADXRS450_Gyro m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetRotation2d()};


  
};
