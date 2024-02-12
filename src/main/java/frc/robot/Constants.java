package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public final class Constants {
  public final class ClawConstants {

    public static final double CONVERT_RATE = 15.74;
    public static final double TOLERANCE = 2;
    public static final double CALIBRATION_SPEED = -0.3;
    public static final double CALIBRATE_THRESHOLD = 0.2;
    public static final double CALIBRATE_ENCODER_DIFF = 0.5;

    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;


    public static final int CLAW_CURRENT_LIMIT = 20;
    public static final int MOTOR_ID = 42;
    public static int MAX_COUNT = 2048;
    public static int DEAD_AXIS_TOLERANCE = 0;
    public static double OPEN_CLAW_POSITION = 34;//27.5
    public static final double VALUES_MULTIPLAYER = 1;
    public static int POSITIVE_RANGE = MAX_COUNT / 4 + DEAD_AXIS_TOLERANCE;
    public static int NEGATIVE_RANGE = MAX_COUNT / 4 - DEAD_AXIS_TOLERANCE;
    public static final PIDController CURRENT_PID = new PIDController(0.000, 0, 0, 0.0077);
    public static final PIDController POSITION_PID = new PIDController(0.01, 0, 0, 0);
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
    public static final double OPEN_SPEED = 0.4;

    public static final double CURRENT_THRESHOLD_CLOSE = 8;
    public static final double MEASURING_TIME = 0.3;
    public static final double QUICK_OPEN_TIME = 1.2;
    public static final int DEADAXIS_ENCODER_MAX_COUNT = 2048;
    public static int MAX_DEGREES = 360;

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
  }
}
}
