// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToTarget extends Command {
  /** Creates a new DriveToTarget. */
  private final LimeLight limelight;
  private final DriveSubsystem driveSubsystem;
  private final PIDController pidController;
  public DriveToTarget(LimeLight limelight, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.driveSubsystem = driveSubsystem;
    pidController = new PIDController(0, 0, 0);
    pidController.setSetpoint(limelight.getTrueDistance());
    pidController.setTolerance(5);
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yspeed = pidController.calculate(limelight.getTrueDistance());
    driveSubsystem.drive(0, yspeed, 0, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0,0,0,true,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pidController.getPositionError()) <= Math.abs(pidController.getPositionTolerance());
  }
}
