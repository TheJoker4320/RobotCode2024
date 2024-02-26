// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.subsystems.DriveSubsystem;

public class PoseEstimatorUtils extends SubsystemBase {
  private static PoseEstimatorUtils poseEstimatorUtils;
  private DriveSubsystem m_DriveSubsystem;
  private LimeLight limelight;
  private SwerveDrivePoseEstimator poseEstimator;
  private Pose2d position;

  public PoseEstimatorUtils(DriveSubsystem m_DriveSubsystem, LimeLight limeLight) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.limelight = limeLight;
    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.DriveConstants.kDriveKinematics,
      m_DriveSubsystem.getPose().getRotation(),
    m_DriveSubsystem.getModulePosition(), 
    limelight.getLimeLightBotPose(),
    VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10)));
  }

  public Pose2d GetPosition(){
    return position;
  }

  @Override
  public void periodic() {
    poseEstimator.update(m_DriveSubsystem.getPose().getRotation(), m_DriveSubsystem.getModulePosition());
    if(limelight.doesLimeLightHaveTargets()){
       poseEstimator.addVisionMeasurement(limelight.getLimeLightBotPose(),
       Timer.getFPGATimestamp() - limelight.getLimeLightTLValue() / 1000 - limelight.getLimeLightCLValue() / 1000); 
    }
    
    position = poseEstimator.getEstimatedPosition();
    double[] pose = new double[3];
    pose[0] = position.getTranslation().getX();
    pose[1] = position.getTranslation().getY();
    pose[2] = position.getRotation().getRadians() *-1;

  }

}
