// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorUtils;

public class DriveToTarget extends Command {
  private PoseEstimatorUtils m_poseEstimator;
  private ProfiledPIDController m_pidControllerProfiledX;
  private ProfiledPIDController m_pidControllerProfiledY;
  private ProfiledPIDController m_pidControllerProfiledOmega;
  private TrapezoidProfile.Constraints m_xConstraints;
  private TrapezoidProfile.Constraints m_yConstraints;
  private TrapezoidProfile.Constraints m_omegaConstraints;
  private DriveSubsystem m_driveSubsystem;
  private LimeLight limeLight;
  private Pose3d goalPose;
  

  /** Creates a new DriveToTarget. */
  public DriveToTarget(DriveSubsystem m_driveSubsystem, PoseEstimatorUtils m_PoseEstimator, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_poseEstimator = m_PoseEstimator;
    this.limeLight = limeLight;

    m_pidControllerProfiledX = new ProfiledPIDController(0, 0, 0, m_xConstraints);
    m_pidControllerProfiledY = new ProfiledPIDController(0, 0, 0, m_yConstraints);
    m_pidControllerProfiledOmega = new ProfiledPIDController(0, 0, 0, m_omegaConstraints);

    m_pidControllerProfiledX.setTolerance(0.2);
    m_pidControllerProfiledY.setTolerance(0.2);
    m_pidControllerProfiledOmega.setTolerance(0.2);
    m_pidControllerProfiledOmega.enableContinuousInput(Math.PI, -Math.PI);

    m_xConstraints = new TrapezoidProfile.Constraints(Constants.TrapezoidProfileConstants.X_MAX_VELOCITY,Constants.TrapezoidProfileConstants.X_MAX_ACC);
    m_yConstraints = new TrapezoidProfile.Constraints(Constants.TrapezoidProfileConstants.Y_MAX_VELOCITY,Constants.TrapezoidProfileConstants.Y_MAX_ACC);
    m_omegaConstraints = new TrapezoidProfile.Constraints(Constants.TrapezoidProfileConstants.OMEGA_MAX_VELOCITY, Constants.TrapezoidProfileConstants.OMEGA_MAX_ACC);

    //TODO: calculate pid and add to constants
    addRequirements(m_driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalPose = Constants.FieldConstants.APRILTAGS.get(limeLight.GetId());
    m_pidControllerProfiledX.setGoal(goalPose.getX());
    m_pidControllerProfiledY.setGoal(goalPose.getY());
    m_pidControllerProfiledOmega.setGoal(goalPose.getRotation().getZ());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // FIXME: move the setPoint half of the robot length towards the field so it'll
    // be where the robot center should be

    double xSpeed = m_pidControllerProfiledX.calculate(m_poseEstimator.GetPosition().getX());
    double ySpeed = m_pidControllerProfiledY.calculate(m_poseEstimator.GetPosition().getY());
    double omega = m_pidControllerProfiledOmega.calculate(m_poseEstimator.GetPosition().getRotation().getRadians());
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
    return m_pidControllerProfiledX.atSetpoint() && m_pidControllerProfiledY.atSetpoint() && m_pidControllerProfiledOmega.atSetpoint();
  }
}
