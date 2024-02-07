package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry jsonDumpNetworkTableEntry = table.getEntry("json");

    

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
    public double getLimeLightTLValue(){
        double tl = table.getEntry("tl").getDouble(0.0);
        SmartDashboard.putNumber("tl", tl);
        return tl;
    }
    public double getLimeLightCLValue(){
        double cl = table.getEntry("cl").getDouble(0.0);
        SmartDashboard.putNumber("cl", cl);
        return cl;
    }

    public double getLimeLightAreaValue() {
        return table.getEntry("ta").getDouble(0.0);
    }
    public Pose3d getLimeLightBotPose(){
        Double[] botpose = table.getEntry("botpose").getDoubleArray(new Double[6]);
        Pose3d pose = new Pose3d(new Translation3d(botpose[0], botpose[1], botpose[2]),
        new Rotation3d(botpose[3], botpose[4], botpose[5]));
        return pose;
    }
    public Pose3d getLimeLightObjectToRobotPose(){
        Double[] targetposerobotspace = table.getEntry("targetpose_robotspace").getDoubleArray(new Double[6]);
        Pose3d pose = new Pose3d(new Translation3d(targetposerobotspace[0], targetposerobotspace[1], targetposerobotspace[2]),
        new Rotation3d(targetposerobotspace[3], targetposerobotspace[4], targetposerobotspace[5]));
        return pose;
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
