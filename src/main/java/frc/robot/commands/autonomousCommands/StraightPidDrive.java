// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class StraightPidDrive extends Command {
  /** Creates a new StraightPidDrive. */

  private final DriveSubsystem m_DriveSubsystem;
  private final PIDController xAxis;
  private final PIDController yAxis;
  private final PIDController thetaAxis;
  private final Pose2d goal;
  private final Timer timer;
  private final int timeout = 4;

  private final int id;

  public StraightPidDrive(DriveSubsystem driveSubsystem, PIDController xAxis, PIDController yAxis, PIDController thetaAxis, Pose2d goal, int id) {
    this.m_DriveSubsystem = driveSubsystem;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.thetaAxis = thetaAxis;
    this.goal = goal;

    this.id = id;

    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_DriveSubsystem.setModulesDirection(0.0);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Pose2d currentPose2d = m_DriveSubsystem.getPose();
    m_DriveSubsystem.drive(/*xAxis.calculate(currentPose2d.getX(), goal.getX()) / DriveConstants.kMaxSpeedMetersPerSecond*/0,
                           yAxis.calculate(currentPose2d.getY(), goal.getY()) / DriveConstants.kMaxSpeedMetersPerSecond,
                           /*thetaAxis.calculate(currentPose2d.getRotation().getRadians(), goal.getRotation().getRadians()) / DriveConstants.kMaxAngularSpeed*/0,
                           true,
                           false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    timer.stop();
    timer.reset();
    m_DriveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose2d = m_DriveSubsystem.getPose();
    boolean xAtSetpoint = Math.abs(currentPose2d.getX() - goal.getX()) < AutoConstants.kToleranceXController;
    boolean yAtSetpoint = Math.abs(currentPose2d.getY() - goal.getY()) < AutoConstants.kToleranceYController;
    boolean thetaAtSetpoint = Math.abs(currentPose2d.getRotation().getRadians() - goal.getRotation().getRadians()) < AutoConstants.kToleranceThetaController;

    boolean timeRanOut = timer.get() >= this.timeout;
    SmartDashboard.putBoolean("Ran out of time - [" + id + "]", timeRanOut);

    return (xAtSetpoint && yAtSetpoint && thetaAtSetpoint) || (timeRanOut);
  }
}
