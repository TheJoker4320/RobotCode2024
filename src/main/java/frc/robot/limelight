package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    

    public double getLimeLightXValue() {
        double x = table.getEntry("tx").getDouble(0.0);
        SmartDashboard.putNumber("tx", x);
        return x;
    }

    public double getLimeLightYValue() {
        double y = table.getEntry("ty").getDouble(0.0);
        SmartDashboard.putNumber("ty", y);
        return y;
    }

    public double getLimeLightAreaValue() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public double getTrueDistance() {
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        double angleToGoalDegrees = Constants.LimeLightConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        // calculate distance
        double distanceFromRobotToGoalCentimeters = ((Constants.LimeLightConstants.GOAL_HEIGHT_CM - Constants.LimeLightConstants.LIMELIGHT_LENSE_HEIGHT_CM)
                / Math.tan(angleToGoalRadians)) - Constants.LimeLightConstants.LL_DISTANCE_FROM_ROBOT_EDGE;
        SmartDashboard.putNumber("Distance to goal", distanceFromRobotToGoalCentimeters);
        return distanceFromRobotToGoalCentimeters ;
    }

    public double getLimeLightMountAngle() {
        double LimeLightAngleDegrees = Math
                .atan((Constants.LimeLightConstants.GOAL_HEIGHT_CM - Constants.LimeLightConstants.LIMELIGHT_LENSE_HEIGHT_CM) / (Constants.LimeLightConstants.distanceFromRobotToGoalCentimetersPreset + Constants.LimeLightConstants.LL_DISTANCE_FROM_ROBOT_EDGE  ))
                + getLimeLightYValue();
        SmartDashboard.putNumber("LimeLight mount angle", LimeLightAngleDegrees);
        return LimeLightAngleDegrees;
    }
}
