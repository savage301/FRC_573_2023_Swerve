// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// The rear modules must be started 180 degrees from those in the front for the motors to drive correctly.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>

#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel,rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,rev::CANSparkMax::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
      m_turningEncoder(turningEncoderChannel) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.

  m_driveEncoder.SetPositionConversionFactor(2 * wpi::numbers::pi * kWheelRadius /
                                     kDriveEncoderResolution);
  m_driveEncoder.SetVelocityConversionFactor(2 * wpi::numbers::pi * kWheelRadius /
                                     kDriveEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
    -units::radian_t(wpi::numbers::pi),units::radian_t(wpi::numbers::pi));
}

// removed const to fix build
frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
         frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()))};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  //const auto state = referenceState;
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.GetPosition()));

/*
    frc::SmartDashboard::PutString("Module Ref Speed", std::to_string(referenceState.speed.value()));
    frc::SmartDashboard::PutString("Module Ref Angle", std::to_string(referenceState.angle.Radians().value()));
    frc::SmartDashboard::PutString("Module Command Speed", std::to_string(state.speed.value()));
    frc::SmartDashboard::PutString("Module Command Angle", std::to_string(state.angle.Radians().value()));
    */
   
  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
    (units::radian_t (m_turningEncoder.GetPosition())), state.angle.Radians());

    frc::SmartDashboard::PutString("Turn Enc Input", std::to_string(m_turningEncoder.GetPosition()));
    frc::SmartDashboard::PutString("Turn State Input", std::to_string(state.angle.Radians().value()));
    frc::SmartDashboard::PutString("Turn Output", std::to_string(turnOutput));
    frc::SmartDashboard::PutString("can coder", std::to_string(m_turningEncoder.GetPosition()));
    

    const auto turnFeedforward = m_turnFeedforward.Calculate(
      units::radians_per_second_t(m_turningPIDController.GetSetpoint().velocity));

  // Set the motor outputs.
  if (m_driveMotor.GetDeviceId() == 1 || m_driveMotor.GetDeviceId() == 7 || m_driveMotor.GetDeviceId() == 4){
    m_driveMotor.SetVoltage(units::volt_t{-driveOutput} );//+ driveFeedforward);
  }
  else{
    m_driveMotor.SetVoltage(units::volt_t{driveOutput} );//+ driveFeedforward);
  }
  //m_driveMotor.SetVoltage(units::volt_t{driveOutput} );//+ driveFeedforward);
  
  if (m_turningMotor.GetDeviceId() == 5 || m_turningMotor.GetDeviceId() == 11){
    m_turningMotor.SetVoltage(units::volt_t{-turnOutput} );//+ turnFeedforward);
  }else{
    m_turningMotor.SetVoltage(units::volt_t{turnOutput} );//+ turnFeedforward);
  }
  //m_turningMotor.SetVoltage(units::volt_t{turnOutput} );//+ turnFeedforward);

  frc::SmartDashboard::PutString("Module Drive Motor Voltage", std::to_string((units::volt_t{driveOutput} + driveFeedforward).value()));
  frc::SmartDashboard::PutString("Module Turn Motor Output Voltage", std::to_string((units::volt_t{turnOutput}).value()));
  frc::SmartDashboard::PutString("Module Turn Motor FF Output Voltage", std::to_string((units::volt_t{turnFeedforward}.value())));
  
}
