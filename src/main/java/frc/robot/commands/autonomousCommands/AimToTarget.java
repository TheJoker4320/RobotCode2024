// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.subsystems.DriveSubsystem;

public class AimToTarget extends Command {
  private final LimeLight limelight;
  private final PIDController pidController;
  private final DriveSubsystem driveSubsystem;
  public AimToTarget(final DriveSubsystem driveSubsystem,final LimeLight limelight) {
    this.limelight = limelight;
    pidController = new PIDController(Constants.LimeLightConstants.AIMING_KP, Constants.LimeLightConstants.AIMING_KI, Constants.LimeLightConstants.AIMING_KD);
    this.driveSubsystem = driveSubsystem;
    pidController.setTolerance(0.5);
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RotValue = pidController.calculate(limelight.getLimeLightXValue());
    driveSubsystem.drive(0,0,RotValue,true,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0,0,0,true,true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("position error", pidController.getPositionError());
    SmartDashboard.putNumber("position tolerance", pidController.getPositionTolerance());
    SmartDashboard.putBoolean("isFinished", pidController.getPositionError() <= pidController.getPositionTolerance());
    return Math.abs(pidController.getPositionError()) <= Math.abs(pidController.getPositionTolerance());
    
  }
}
