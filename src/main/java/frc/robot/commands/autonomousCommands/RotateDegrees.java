// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDegrees extends Command {
  /** Creates a new RotateDegrees. */
  private DriveSubsystem driveSubsystem;
  private double desiredDegree;
  private PIDController pidController;
  private Timer timer;
  public RotateDegrees(DriveSubsystem driveSubsystem, double desiredDegree) {
    this.driveSubsystem = driveSubsystem;
    this.desiredDegree = desiredDegree;
    pidController = new PIDController(0.04, 0, 0);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(1);
    timer = new Timer();
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(desiredDegree);
    SmartDashboard.putBoolean("Finished", false);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("RotateHeading", driveSubsystem.getHeading());
    SmartDashboard.putNumber("RotateSetpoint", pidController.getSetpoint());
    double output = pidController.calculate(driveSubsystem.getHeading());
    output *= -1;
    output = output > 0.3 ? 0.3 : output;
    output = output < -0.3 ? -0.3 : output;
    driveSubsystem.drive(0, 0, output, true, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Finished", true);
    timer.stop();
    driveSubsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint() || timer.get() >= 5;
  }
}
