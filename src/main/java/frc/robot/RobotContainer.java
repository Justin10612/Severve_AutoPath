// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.SwerveDrivebase;

public class RobotContainer {
  private final SwerveDrivebase swerveSubsystem = new SwerveDrivebase();

  private final Joystick driveJoystick = new Joystick(0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new ManualDrive(
      swerveSubsystem,
      () -> -driveJoystick.getRawAxis(JoystickConstants.kJoystickXspeedAxis),
      () -> driveJoystick.getRawAxis(JoystickConstants.kJoystickYspeedAxis), 
      () -> driveJoystick.getRawAxis(JoystickConstants.kJoystickTurnSpeedAxis), 
      () -> driveJoystick.getRawButton(JoystickConstants.kJoystickFildOrientedBtn)));
    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
