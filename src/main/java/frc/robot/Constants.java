package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public final class Constants {
  public static final class CollectorConstants {
    public static final int COLLECTOR_PORT = 15;
    public static final int LIMIT_SWITCH_CHANNEL = 0;// TODO: correct the channel

    public static final double COLLECTOR_SPEED = 0.7;
  }

  public static class ModuleConstants
  {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kFreeSpeedRpm = 5676;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60; 
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps 
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; 
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second 

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(30);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(30);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 8; 
    public static final int kRearLeftDrivingCanId = 6; 
    public static final int kFrontRightDrivingCanId = 2; 
    public static final int kRearRightDrivingCanId = 4; 

    public static final int kFrontLeftTurningCanId = 7; 
    public static final int kRearLeftTurningCanId = 5; 
    public static final int kFrontRightTurningCanId = 1; 
    public static final int kRearRightTurningCanId = 3; 

    public static final boolean kGyroReversed = false;
  }

  public static final class ShooterConstants{
    public static final int SHOOTER_MASTER_PORT = 14;
    public static final int SHOOTER_SLAVE_PORT = 13;
    public static final double SHOOT_SPEED = 1;
  }

  public final class ClawConstants {

    public static final double CONVERT_RATE = 360;
    public static final double TOLERANCE = 2;
    public static final double CALIBRATION_SPEED = -0.3;
    public static final double CALIBRATE_THRESHOLD = 0.2;
    public static final double CALIBRATE_ENCODER_DIFF = 0.5;

    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;


    public static final int CLAW_CURRENT_LIMIT = 20;
    public static final int MOTOR_ID1 = 9;
    public static final int MOTOR_ID2 = 10;
    public static int MAX_COUNT = 2048;
    public static int DEAD_AXIS_TOLERANCE = 0;
    public static double OPEN_CLAW_POSITION = 34;//27.5
    public static final double VALUES_MULTIPLAYER = 1;
    public static int POSITIVE_RANGE = MAX_COUNT / 4 + DEAD_AXIS_TOLERANCE;
    public static int NEGATIVE_RANGE = MAX_COUNT / 4 - DEAD_AXIS_TOLERANCE;


    public static final PIDController CURRENT_PID = new PIDController(0.000, 0, 0, 0.0077);
    public static final PIDController POSITION_PID = new PIDController(0.01, 0, 0, 2);
    public static final double MIN_VALUE = -1;
    public static final double MAX_VALUE = 1;  
    public static final double I_ZONE = 1.5;
    public static final double FULL_OPEN_CAP = 8 * VALUES_MULTIPLAYER;
    public static final double FAST_CLOSE_SPEED = -0.5;
    public static final double CLOSE_SPEED = -1;
    public static final double CLOSE_CURRENT_CUBE = -15;
    public static final double THRESHOLD = 0.5;
    public static final double CLOSE_POSITION = 0.5;
    public static final double CALIBRATE_TIME = 1.5;
    public static final double CLOSE_CURRENT_CONE = -15;
    public static final double OPEN_SPEED = 0.2;

    public static final double CURRENT_THRESHOLD_CLOSE = 8;
    public static final double MEASURING_TIME = 0.3;
    public static final double QUICK_OPEN_TIME = 1.2;
    public static final int DEADAXIS_ENCODER_MAX_COUNT = 2048;
    public static int MAX_DEGREES = 80;
    public static int MIN_DEGREES = 5;
    public static double DESIRED_DEGREE = 35;
    public static final double ENCODER_OFFSET = 40;

    public static double MIN_NOT_SAVE_DEGREES = -5;
    public static double MAX_NOT_SAVE_DEGREES = 35;

    public static final double CLAW_TOUCH_OBJECT_SECONDS = 0.2;

    public static int ENCODER_TO_DEGREES(int encoderCount) {
        return encoderCount * (MAX_DEGREES / DEADAXIS_ENCODER_MAX_COUNT);
    }

    public static int DEGREES_TO_ENCODER(int degrees) {
        return degrees * (DEADAXIS_ENCODER_MAX_COUNT / MAX_DEGREES);
    }
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kOperatorControllerPort = 1;

      public static final double kDriveDeadband = 0.05;

      public static final int kCollectBtn = XboxController.Button.kA.value;
      public static final int kEjectBtn = XboxController.Button.kB.value;
      public static final int kShootBtn = XboxController.Button.kY.value;
      public static final int kCollectToShootBtn = XboxController.Button.kX.value;
    }
  }
}

