package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


  /**
   * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
   * constants. This class should not be used for any other purpose. All constants should be declared
   * globally (i.e. public static). Do not put anything functional in this class.
   *
   * <p>It is advised to statically import this class (or one of its inner classes) wherever the
   * constants are needed, to reduce verbosity.
   */
public final class Constants {
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
    public static final double ROBOT_LENGTH =30;

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
    public static final int SHOOTER_ENCODER_PORT_A = 1;
    public static final int SHOOTER_ENCODER_PORT_B = 2;
    public static final double SHOOTER_DISTANCE_PER_PULSE = 1.0/2048;
	public static final double kP = 0.08;// FIXME: correct the value
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public final class ArmConstants {

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

    public static double CURRENTPID_P = 0.075;
    public static double CURRENTPID_I = 0;
    public static double CURRENTPID_D = 0;
    public static double CUREENTPID_TOLORANCE = 0.5;

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
    public static int MAX_DEGREES = 90;
    public static int MIN_DEGREES = 3;
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
}
public static class DistanceToAngle{
  public static final double m = 2.169;
  public static final double constant = 19.347;
}
public static class OperatorConstants {
  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;

  public static final double kDriveDeadband = 0.05;

  public static final int kCollectBtn = PS4Controller.Button.kCross.value;
  public static final int kEjectBtn = PS4Controller.Button.kSquare.value;
  public static final int kShootBtn = PS4Controller.Button.kR1.value;
  public static final int kRaiseBtn = PS4Controller.Button.kR2.value;
  public static final int kLowerBtn = PS4Controller.Button.kL2.value;
  public static final int kAimArmToSpeaker = PS4Controller.Button.kL1.value;
  
}
public static final class CollectorConstants {
public static final int COLLECTOR_PORT = 15;
public static final int LIMIT_SWITCH_CHANNEL = 0;// TODO: correct the channel

public static final double COLLECTOR_SPEED = 0.7;
}

  public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 2; // Can go up to 3, for safety and accuracy only 2 at the
                                                               // current moment
      public static final double kMaxAccelerationMetersPerSecondSquared = 2; // Can go up to 3, for safety and accuracy
                                                                             // only 2 at the current moment
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // The distance from the center of the robot to the farthers module
    public static final double kSwerveDriveRadius = (DriveConstants.kTrackWidth * Math.sqrt(2)) / 2.0;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class LimeLightConstants {
      public static final double AIMING_KP = 0.004;
      public static final double AIMING_KI = 0.003;
      public static final double AIMING_KD = 0;
      public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0; // how many degrees back is your limelight rotated
                                                                    // from perfectly vertical
      public static final double LIMELIGHT_LENSE_HEIGHT_CM = 27; // distance from the center of the Limelight lens to
                                                                 // the floor
      public static final double GOAL_HEIGHT_CM = 74.6; // distance from the target to the floor

    public static final double LL_DISTANCE_FROM_ROBOT_EDGE = 16.2;
    public static final double distanceFromRobotToGoalCentimetersPreset = 216.7; // distance from target in to calibrate
                                                                                 // the LimeLight mount angle

}

public static final class FieldConstants {
      public static final double FIELD_CENTER_X = 8.308467;
      public static final double FIELD_CENTER_Y = 4.098925;

      public static final Map<Integer, Pose3d> APRILTAGS = Map.ofEntries(
              Map.entry(1,
                      new Pose3d(
                              FIELD_CENTER_X + 6.808597,
                              FIELD_CENTER_Y + -3.859403,
                              1.355852,
                              new Rotation3d(0.0, 0.0, Math.PI * 2 / 3))),
              Map.entry(2,
                      new Pose3d(
                              FIELD_CENTER_X + 7.914259,
                              FIELD_CENTER_Y + -3.221609,
                              1.355852,
                              new Rotation3d(0.0, 0.0, Math.PI * 2 / 3))),
              Map.entry(3,
                      new Pose3d(
                              FIELD_CENTER_X + 8.308467,
                              FIELD_CENTER_Y + 0.877443, // FIRST's diagram has a typo (it says 147.19)
                              1.451102,
                              new Rotation3d(0.0, 0.0, Math.PI))),
              Map.entry(4,
                      new Pose3d(
                              FIELD_CENTER_X + 8.308467,
                              FIELD_CENTER_Y + 1.442593,
                              1.451102,
                              new Rotation3d(0.0, 0.0, Math.PI))),
              Map.entry(5,
                      new Pose3d(
                              FIELD_CENTER_X + 6.429883,
                              FIELD_CENTER_Y + 4.098925,
                              1.355852,
                              new Rotation3d(0, 0, -Math.PI / 2))),
              Map.entry(6,
                      new Pose3d(
                              FIELD_CENTER_X + -6.429375,
                              FIELD_CENTER_Y + 4.098925,
                              1.355852,
                              new Rotation3d(0, 0, -Math.PI / 2))),
              Map.entry(7,
                      new Pose3d(
                              FIELD_CENTER_X + -8.308975,
                              FIELD_CENTER_Y + 1.442593,
                              1.451102,
                              new Rotation3d(0, 0, 0))),
              Map.entry(8,
                      new Pose3d(
                              FIELD_CENTER_X + -8.308975,
                              FIELD_CENTER_Y + 0.877443,
                              1.451102,
                              new Rotation3d(0, 0, 0))),
              Map.entry(9,
                      new Pose3d(
                              FIELD_CENTER_X + -7.914767,
                              FIELD_CENTER_Y + -3.221609,
                              1.355852,
                              new Rotation3d(0, 0, Math.PI / 3))),
              Map.entry(10,
                      new Pose3d(
                              FIELD_CENTER_X + -6.809359,
                              FIELD_CENTER_Y + -3.859403,
                              1.355852,
                              new Rotation3d(0, 0, Math.PI / 3))),
              Map.entry(11,
                      new Pose3d(
                              FIELD_CENTER_X + 3.633851,
                              FIELD_CENTER_Y + -0.392049,
                              1.3208,
                              new Rotation3d(0, 0, -Math.PI / 3))),
              Map.entry(12,
                      new Pose3d(
                              FIELD_CENTER_X + 3.633851,
                              FIELD_CENTER_Y + 0.393065,
                              1.3208,
                              new Rotation3d(0, 0, Math.PI / 3))),
              Map.entry(13,
                      new Pose3d(
                              FIELD_CENTER_X + 2.949321,
                              FIELD_CENTER_Y + -0.000127,
                              1.3208,
                              new Rotation3d(0, 0, Math.PI))),
              Map.entry(14,
                      new Pose3d(
                              FIELD_CENTER_X + -2.950083,
                              FIELD_CENTER_Y + -0.000127,
                              1.3208,
                              new Rotation3d(0, 0, 0))),
              Map.entry(15,
                      new Pose3d(
                              FIELD_CENTER_X + -3.629533,
                              FIELD_CENTER_Y + 0.393065,
                              1.3208,
                              new Rotation3d(0, 0, Math.PI * 2 / 3))),
              Map.entry(16,
                      new Pose3d(
                              FIELD_CENTER_X + -3.629533,
                              FIELD_CENTER_Y + -0.392049,
                              1.3208,
                              new Rotation3d(0, 0, -Math.PI * 2 / 3))));
  }

  public static final class TrapezoidProfileConstants{

        public static final int X_MAX_VELOCITY = 3;
        public static final int X_MAX_ACC = 2;
        public static final int Y_MAX_VELOCITY = 3;
        public static final int Y_MAX_ACC = 2;
        public static final int OMEGA_MAX_VELOCITY = 8;
        public static final int OMEGA_MAX_ACC = 8;
        
        //constraints from video





  }
}

