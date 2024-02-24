// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;


/** Add your docs here. */
public class FieldPosUtils {

    public static Pose2d RobotToAmp(){

        if(DriverStation.getAlliance().get() == Alliance.Blue){
            Pose3d targetPose3d = Constants.FieldConstants.APRILTAGS.get("6");
            Pose2d targPose2d = new Pose2d(targetPose3d.getX(),targetPose3d.getY() - Constants.DriveConstants.ROBOT_LENGTH/2 , new Rotation2d(targetPose3d.getZ() + Math.PI));
            return targPose2d;
        }

        Pose3d targetPose3d = Constants.FieldConstants.APRILTAGS.get("5");
        Pose2d targPose2d = new Pose2d(targetPose3d.getX(),targetPose3d.getY() - Constants.DriveConstants.ROBOT_LENGTH/2 , new Rotation2d(targetPose3d.getZ() + Math.PI));
        return targPose2d;

        }
    }


