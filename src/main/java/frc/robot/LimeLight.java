package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimeLightConstants;

public class LimeLight {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public boolean doesLimeLightHaveTargets(){
        return table.getEntry("tv").getInteger(0) == 1;
    }

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

    public int GetId(){
        return (int) table.getEntry("tid").getInteger(0);
    }

    public Pose2d getLimeLightBotPose(){

        double[] botpose;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[1]);
        }
        else{
            botpose = table.getEntry("botpose_wpired").getDoubleArray(new double[1]);
        }

        double bot_x = botpose[0];
        double bot_y = botpose[1];
        double rotation_z = (botpose[5] + 360) % 360;
        return new Pose2d(
            new Translation2d(bot_x, bot_y), Rotation2d.fromDegrees(rotation_z)
        );
        //Pose3d pose = new Pose3d(new Translation3d(botpose[0], botpose[1], botpose[2]),
        //new Rotation3d(botpose[3], botpose[4], botpose[5]));
        //return pose;
    }
    public Pose3d getLimeLightObjectToRobotPose(){
        if(!doesLimeLightHaveTargets())
            return new Pose3d();
        Double[] targetposerobotspace = table.getEntry("targetpose_robotspace").getDoubleArray(new Double[1]);
        Pose3d pose = new Pose3d(new Translation3d(targetposerobotspace[0], targetposerobotspace[1], targetposerobotspace[2]),
        new Rotation3d(targetposerobotspace[3], targetposerobotspace[4], targetposerobotspace[5]));
        return pose;
    }

    public static double getTrueDistance() {
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
    public static double GetArmAngle(){
        double distance = 10 * getTrueDistance();
        return -28.8 + 0.0707 * distance - 4.56E-5 * Math.pow(distance, 2) + 1.51E-8 * Math.pow(distance, 3) - 1.89E-12 * Math.pow(distance, 4);
    }
}
