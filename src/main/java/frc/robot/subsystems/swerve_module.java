// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SModuleConstants;

public class swerve_module extends SubsystemBase {
  // Spark Controller
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;
  // Motor Encoder
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;
  // Turning PID Controller
  private final PIDController turningPIDController;
  // CAN Encoder
  private final CANcoder absoluteEncoder;
  // Variable
  private double angleSetPoint;
  private double angleMeasurement;

  public swerve_module(int driveMotorID, int turnMotorID, boolean driveMotorInverse, boolean turnMotorInverse, int absoluteEncoderID, double absoluteEncoderOffsetDegree){
    // CANcoder
    absoluteEncoder = new CANcoder(absoluteEncoderID);
    var cancoderCfg = new CANcoderConfiguration();
    cancoderCfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderCfg.MagnetSensor.MagnetOffset = absoluteEncoderOffsetDegree;
    absoluteEncoder.getConfigurator().apply(cancoderCfg);
    // MotorController
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    // Restore Factory Defaults
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();
    // Motor Inverse
    driveMotor.setInverted(driveMotorInverse);
    turnMotor.setInverted(turnMotorInverse);
    // IdleMode
    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);
    // Burn Flash
    driveMotor.burnFlash();
    turnMotor.burnFlash();
    // Get Motor Encoder
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getEncoder();
    // Setting Position Conversion Factor for driveMotor
    driveEncoder.setPositionConversionFactor(SModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(SModuleConstants.kDriveEncoderRPM2MeterPerSec);
    // Setting Velocity Conversion Factor for turnMotor
    turningEncoder.setPositionConversionFactor(SModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(SModuleConstants.kTurningEncoderRPM2RadPerSec);
    // Turning PID
    turningPIDController = new PIDController(SModuleConstants.kTurnMotorkP, 0, 0);
    turningPIDController.enableContinuousInput(-180, 180);
    // Reset Encoder
    resetEncoders();
  }

  // Get Drive Motor Encoder Position
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }
  // Get Drive Motor Velocity
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }
  // Get SwerveModule Absolute Position
  public double getTurningPosition(){
    return absoluteEncoder.getAbsolutePosition().getValue() * 360;
  }
  // Get Turn Motor Encoder Position
  public double getTurnintEncoderPosition(){
    return turningEncoder.getPosition();
  }
  // Get Turn Motor Velocity
  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }
  // Reset Encoder Position to 0
  public void resetEncoders(){
    driveEncoder.setPosition(0);
  turningEncoder.setPosition(0);
  }
  // Get SwerveModule State
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPosition()));
  }
  // Get SwerveModule Position
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningPosition()));
  }
  // Set SwerveModule Stete
  public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond)<0.001){
      stopModule();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    // Set Motor
    driveMotor.set(state.speedMetersPerSecond);
    angleSetPoint = state.angle.getDegrees();
    angleMeasurement = getState().angle.getDegrees();
    turnMotor.set(turningPIDController.calculate(angleMeasurement, angleSetPoint));
  }
  // Stop Module Motor
  public void stopModule(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("angleMeasurement", angleMeasurement);
    // SmartDashboard.putNumber("angleSetpoint", angleSetPoint);
  }
}
