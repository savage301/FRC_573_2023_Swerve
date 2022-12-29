// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include "ctre/Phoenix.h"

#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int turningEncoderChannel);
  frc::SwerveModuleState GetState();
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr int kDriveEncoderResolution = 4096*6.75;
  static constexpr int kSteerEncoderResolution = 4096;

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::numbers::pi * 2_rad_per_s_sq;  // radians per second^2

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;


  rev::SparkMaxRelativeEncoder m_driveEncoder;
  ctre::phoenix::sensors::CANCoder m_turningEncoder;

  frc2::PIDController m_drivePIDController{1.0, 0, 0};


  //double pval = frc::SmartDashboard::GetNumber("turn_PVal",1.0);

  frc2::PIDController m_turningPIDController{12/1.5, 0, 0};
  
  /*frc::ProfiledPIDController<units::radians> m_turningPIDController{
      25.0,
      0.0,
      0.0,
      frc::TrapezoidProfile<units::radians>::Constraints{kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
*/
  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};
};
