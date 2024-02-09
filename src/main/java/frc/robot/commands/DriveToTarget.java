// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorUtils;

public class DriveToTarget extends Command {
  private PoseEstimatorUtils m_poseEstimator;
  private PIDController m_pidControllerX;
  private PIDController m_pidControllerY;
  private PIDController m_pidControllerOmega;
  private DriveSubsystem m_driveSubsystem;
  private LimeLight limeLight;
  private Pose3d goalPose;
  

  /** Creates a new DriveToTarget. */
  public DriveToTarget(DriveSubsystem m_driveSubsystem, PoseEstimatorUtils m_PoseEstimator, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_poseEstimator = m_PoseEstimator;
    this.limeLight = limeLight;
    this.m_pidControllerX = new PIDController(0, 0, 0);
    this.m_pidControllerY = new PIDController(0, 0, 0);
    this.m_pidControllerOmega = new PIDController(0, 0, 0);
    m_pidControllerX.setTolerance(0.05);
    m_pidControllerY.setTolerance(0.05);
    m_pidControllerOmega.setTolerance(Units.degreesToRadians(2.5));

    //TODO: calculate pid and add to constants
    addRequirements(m_driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalPose = Constants.FieldConstants.APRILTAGS.get(limeLight.GetId());
    m_pidControllerX.setSetpoint(goalPose.getX());
    m_pidControllerY.setSetpoint(goalPose.getY());
    m_pidControllerOmega.setSetpoint(goalPose.getRotation().getZ());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // FIXME: move the setPoint half of the robot length towards the field so it'll
    // be where the robot center should be
    double xSpeed = m_pidControllerX.calculate(m_poseEstimator.GetPosition().getX());
    double ySpeed = m_pidControllerX.calculate(m_poseEstimator.GetPosition().getY());
    double omega = m_pidControllerX.calculate(m_poseEstimator.GetPosition().getRotation().getRadians());
    m_driveSubsystem.drive(xSpeed,ySpeed,omega,true,true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0,0,0,true,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidControllerX.atSetpoint() && m_pidControllerY.atSetpoint() && m_pidControllerOmega.atSetpoint();
  }
}
