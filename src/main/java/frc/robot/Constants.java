package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    // Hardware ID
    public static final int kGyroID = 10;
    public static final class JoystickConstants{
        public static final double kJoystickDeadband = 0.05;
        // Axis
        public static final int kJoystickXspeedAxis = 1;
        public static final int kJoystickYspeedAxis = 0;
        public static final int kJoystickTurnSpeedAxis = 4;
        // Buttons
        public static final int kJoystickFildOrientedBtn = 4;
        public static final int kJoystickManualResetHeadingBtn = 1;
    }

    // Constants for SwerveModule   
    public static final class SModuleConstants{
        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 1.0/(150/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio*Math.PI*kWheelDiameter;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio*2*Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60.0;
        public static final double kTurnMotorkP = 0.005;
        public static final double kMaxDriveMotorSpeed = 4.0;
    }
    // Auto Path Constants
    public static final class AutoConstants{
        public static final double kMaxAutoSpeedMetersPerSec = 4.5;
        public static final double kAutoDriveMaxAngularSpeedMetersPerSec = 0.25;
    }
    // Constants for Drivebase
    public static final class SwerveConstants{
        // Max Drive Speed
        public static final double kTeleDriveMaxSpeedMetersPerSec = 0.25;
        public static final double kTeleDriveMaxAngularSpeedMetersPerSec = 0.25;
        // Slew Rate
        public static final double kLinearSlewRate = 4;
        public static final double kAngularSlewRate = 4;
        // Drivebase
        public static final double kRobotLength = 0.66;
        public static final double kRobotWidth = 0.66;
        public static final double kDrivebaseRadius = 0.47;
        // CAN ID
        public static final int leftFrontDriveID = 7;
        public static final int leftFrontTurningID = 8;
        public static final int rightFrontDriveID = 1;
        public static final int rightFrontTurningID = 2;  
        public static final int leftRearDriveID = 5;
        public static final int leftRearTurningID = 6;
        public static final int rightRearDriveID = 3;
        public static final int rightRearTurningID = 4;

        public static final int leftFrontCANCoderID = 14;
        public static final int rightFrontCANCoderID = 11;
        public static final int leftRearCANCoderID = 13;
        public static final int rightRearCANCoderID = 12;

        public static final boolean leftFrontDriveMotorReversed = true;
        public static final boolean leftFrontTurningMotorReversed = true;
        public static final boolean rightFrontDriveMotorReversed = true;
        public static final boolean rightfrontTurningMotorReversed = true;
        public static final boolean leftRearDriveMotorreversed = false;
        public static final boolean leftRearTurningMotorReversed = true;
        public static final boolean rightRearDriveMotorReversed = false;
        public static final boolean rightRearTurningMotorReversed = true;

        public static final double leftFrontOffset = 135.0/180;
        public static final double rightFrontOffset = -40.0/180;
        public static final double leftRearOffset = -37.0/180;
        public static final double rightRearOffset = 104.0/180;
        
        public static double joysickValue(double value, double mineOutput){
            if(Math.abs(value) < mineOutput) return 0;
            else return value;
        }

        //front left, front right, rear left, rear right
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(kRobotLength/2, kRobotWidth/2), 
            new Translation2d(kRobotLength/2, -kRobotWidth/2), 
            new Translation2d(-kRobotLength/2, kRobotWidth/2),
            new Translation2d(-kRobotLength/2, -kRobotWidth/2)
        );
    }
}
