// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
  /*auto output = frc::ChassisSpeeds{xSpeed,ySpeed,rot};

  frc::SmartDashboard::PutNumber("VX",output.vx.value());
  */
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  frc::SmartDashboard::PutString("FL Desired Speed", std::to_string(fl.speed.value()));
  frc::SmartDashboard::PutString("FR Desired Speed", std::to_string(fr.speed.value()));
  frc::SmartDashboard::PutString("BL Desired Speed", std::to_string(bl.speed.value()));
  frc::SmartDashboard::PutString("BR Desired Speed", std::to_string(br.speed.value()));
  
  frc::SmartDashboard::PutString("FL Desired Angle", std::to_string(fl.angle.Radians().value()));
  frc::SmartDashboard::PutString("FR Desired Angle", std::to_string(fr.angle.Radians().value()));
  frc::SmartDashboard::PutString("BL Desired Angle", std::to_string(bl.angle.Radians().value()));
  frc::SmartDashboard::PutString("BR Desired Angle", std::to_string(br.angle.Radians().value()));
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(), m_frontLeft.GetState(),
                    m_frontRight.GetState(), m_backLeft.GetState(),
                    m_backRight.GetState());
                    
    frc::SmartDashboard::PutString("FL Current Speed", std::to_string(m_frontLeft.GetState().speed.value()));
    frc::SmartDashboard::PutString("FR Current Speed", std::to_string(m_frontRight.GetState().speed.value()));
    frc::SmartDashboard::PutString("BL Current Speed", std::to_string(m_backLeft.GetState().speed.value()));
    frc::SmartDashboard::PutString("BR Current Speed", std::to_string(m_backRight.GetState().speed.value()));
                 
    frc::SmartDashboard::PutString("FL Current Angle", std::to_string(m_frontLeft.GetState().angle.Radians().value()));
    frc::SmartDashboard::PutString("FR Current Angle", std::to_string(m_frontRight.GetState().angle.Radians().value()));
    frc::SmartDashboard::PutString("BL Current Angle", std::to_string(m_backLeft.GetState().angle.Radians().value()));
    frc::SmartDashboard::PutString("BR Current Angle", std::to_string(m_backRight.GetState().angle.Radians().value()));
    
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_odometry.ResetPosition(pose,m_gyro.GetRotation2d());

}

frc::Pose2d Drivetrain::GetPose() const {
  return m_odometry.GetPose();
}