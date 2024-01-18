// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrivebase extends SubsystemBase {
  private final swerve_module leftFrontModule, rightFrontModule, leftRearModule, rightRearModule;
  private final Pigeon2 gyro = new Pigeon2(Constants.kGyroID);
  // private SwerveDriveOdometry m_odometry;
  
  public SwerveDrivebase(){
    // Setting Up Gyro
    var gyroCfg = new Pigeon2Configuration();
    gyroCfg.MountPose.MountPoseYaw = 22;
    gyro.getConfigurator().apply(gyroCfg);
    /* Zeroing Gyro */
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
        SmartDashboard.putNumber("zeroing", 1);
      }catch(Exception e){
      }
    }).start();
    // Setting Each Module
    leftFrontModule = new swerve_module(
      SwerveConstants.leftFrontDriveID, 
      SwerveConstants.leftFrontTurningID, 
      SwerveConstants.leftFrontDriveMotorReversed, 
      SwerveConstants.leftFrontTurningMotorReversed, 
      SwerveConstants.leftFrontCANCoderID, 
      SwerveConstants.leftFrontOffset);
    rightFrontModule = new swerve_module(
      SwerveConstants.rightFrontDriveID,
      SwerveConstants.rightFrontTurningID,
      SwerveConstants.rightFrontDriveMotorReversed, 
      SwerveConstants.rightfrontTurningMotorReversed, 
      SwerveConstants.rightFrontCANCoderID, 
      SwerveConstants.rightFrontOffset);
    leftRearModule = new swerve_module(
      SwerveConstants.leftRearDriveID, 
      SwerveConstants.leftRearTurningID, 
      SwerveConstants.leftRearDriveMotorreversed, 
      SwerveConstants.leftRearTurningMotorReversed, 
      SwerveConstants.leftRearCANCoderID, 
      SwerveConstants.leftRearOffset);
    rightRearModule = new swerve_module(
      SwerveConstants.rightRearDriveID, 
      SwerveConstants.rightRearTurningID, 
      SwerveConstants.rightRearDriveMotorReversed, 
      SwerveConstants.rightRearTurningMotorReversed, 
      SwerveConstants.rightRearCANCoderID, 
      SwerveConstants.rightRearOffset);
    // Setting Odometer
    // m_odometry = new SwerveDriveOdometry(
    //   swerveKinematics, 
    //   gyro.getRotation2d(), 
    //   getModulePosition());
  }
  // Reset Gyro Heading
  public void zeroHeading(){
    gyro.reset();
  }
  // Get Gyro Heading
  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);  // This makes angle_value be limited between +-360.
  }
  // Get Rotation2d
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }
  // Setting Module State
  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SModuleConstants.kMaxDriveMotorSpeed); // Normalize Each wheel to same speed.
    leftFrontModule.setDesiredState(desiredStates[0]);
    rightFrontModule.setDesiredState(desiredStates[1]);
    leftRearModule.setDesiredState(desiredStates[2]);
    rightRearModule.setDesiredState(desiredStates[3]);
    SmartDashboard.putNumber("m1_drive", desiredStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("m1_turn", desiredStates[0].angle.getDegrees());
  }
  // Stop Each Module
  public void stopModules(){
    leftFrontModule.stopModule();
    leftRearModule.stopModule();
    rightFrontModule.stopModule();
    rightRearModule.stopModule();
  }

  public SwerveModulePosition[] getModulePosition(){
    return new SwerveModulePosition[]{
      leftFrontModule.getPosition(),
      rightFrontModule.getPosition(),
      leftRearModule.getPosition(),
      rightRearModule.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      leftFrontModule.getState(),
      rightFrontModule.getState(),
      leftRearModule.getState(),
      rightRearModule.getState()
    };
  }

  // public Pose2d getPose(){
  //     return m_odometry.getPoseMeters();
  // }

  // public void setPose(Pose2d pose){
  //     m_odometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
  // }

  @Override
  public void periodic() {
  }
}
