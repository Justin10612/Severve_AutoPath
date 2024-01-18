// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveConstants;

public class ManualDrive extends Command {
  private final SwerveDrivebase m_SwerveDrivebase;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, turningSpeedFunc;
  private final Supplier<Boolean> fieldOrientedFunc;
  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  /** Creates a new ManualDrive. */
  public ManualDrive(SwerveDrivebase _swerveSubsystem, 
              Supplier<Double> xSpeedFunction, 
              Supplier<Double> ySpeedFunction, 
              Supplier<Double> turningSpeedFunction, 
              Supplier<Boolean> fieldOrientedFunction){
    this.m_SwerveDrivebase = _swerveSubsystem;
    this.xSpeedFunc = xSpeedFunction;
    this.ySpeedFunc = ySpeedFunction;
    this.turningSpeedFunc = turningSpeedFunction;
    this.fieldOrientedFunc = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(SwerveConstants.kLinearSlewRate);
    this.yLimiter = new SlewRateLimiter(SwerveConstants.kLinearSlewRate);
    this.turnLimiter = new SlewRateLimiter(SwerveConstants.kAngularSlewRate);
    addRequirements(m_SwerveDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get Joystick Value
    double xSpeed = xSpeedFunc.get();
    double ySpeed = ySpeedFunc.get();
    double turnSpeed = turningSpeedFunc.get();
    // Apply deadband
    xSpeed = Math.abs(xSpeed) > JoystickConstants.kJoystickDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > JoystickConstants.kJoystickDeadband ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > JoystickConstants.kJoystickDeadband ? turnSpeed : 0.0;
    // Limit the rate of changing
    xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSec;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.kTeleDriveMaxSpeedMetersPerSec;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.kTeleDriveMaxAngularSpeedMetersPerSec;
    // Construct desire chassicSpeed
    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunc.get()){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turnSpeed, m_SwerveDrivebase.getRotation2d());
    }else{
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }
    // Convert ChassisSpeed into individual module state via Kinematics
    SwerveModuleState[] moduleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    // Output each module states to wheels
    m_SwerveDrivebase.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveDrivebase.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
