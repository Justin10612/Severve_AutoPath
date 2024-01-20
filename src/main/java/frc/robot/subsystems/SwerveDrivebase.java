// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrivebase extends SubsystemBase {
  private final swerve_module leftFrontModule, rightFrontModule, leftRearModule, rightRearModule;
  private final Pigeon2 gyro = new Pigeon2(Constants.kGyroID);
  private SwerveDriveOdometry odometer;
  // Visuallze robot position on the Field
  private Field2d m_field = new Field2d();
  
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
    odometer = new SwerveDriveOdometry(
      SwerveConstants.swerveKinematics,  
      gyro.getRotation2d(), // The real heading of robot
      getModulePosition()); // Each heading of Modules 
    // Visualize Robot position on the field
    SmartDashboard.putData("Field", m_field);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      this::getPose,                // Robot pose supplier
      this::setPose,                // Method to reset odometry (will be called if your auto has a starting pose)
      this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative,     // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        AutoConstants.kMaxAutoSpeedMetersPerSec,      // Max module speed, in m/s
        SwerveConstants.kDrivebaseRadius,     // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig()   // Default path replanning config. See the API for the options here
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this // Reference to this subsystem to set requirements
    );
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
  // Get relative chassis speed
  public ChassisSpeeds getSpeeds() {
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }
  //
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
  // Get Pose from Odometer
  public Pose2d getPose(){
      return odometer.getPoseMeters();
  }
  // Align the Odometer pose with real robot pose
  public void setPose(Pose2d pose){
      odometer.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
  }

  @Override
  public void periodic() {
    // Update odometer position
    odometer.update(
      getRotation2d(),
      getModulePosition());
    // 
    m_field.setRobotPose(odometer.getPoseMeters());
  }
}
