// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimeLight;

public class PoseEstimating extends SubsystemBase {
  private DriveSubsystem m_DriveSubsystem;
  private LimeLight limelight;
  private SwerveDrivePoseEstimator poseEstimator;
  public PoseEstimating() {
    m_DriveSubsystem = new DriveSubsystem();
    limelight = new LimeLight();
    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.DriveConstants.kDriveKinematics,
      m_DriveSubsystem.getPose().getRotation(),
    m_DriveSubsystem.getModulePosition(), 
    limelight.getLimeLightBotPose().toPose2d());
  }

  @Override
  public void periodic() {
    poseEstimator.update(m_DriveSubsystem.getPose().getRotation(), m_DriveSubsystem.getModulePosition());
    poseEstimator.addVisionMeasurement(limelight.getLimeLightObjectToRobotPose().toPose2d(),
    Timer.getFPGATimestamp() - limelight.getLimeLightTLValue() / 1000 - limelight.getLimeLightCLValue() / 1000); 
    
    
    Pose2d position = poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("estimated x", position.getTranslation().getX());
    SmartDashboard.putNumber("estimated y", position.getTranslation().getY());
    SmartDashboard.putNumber("estimated degree", position.getRotation().getDegrees());
    SmartDashboard.putNumber("estimated rotation", position.getRotation().getRotations());
  }
}
