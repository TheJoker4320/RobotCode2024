// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

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
    pidController = new PIDController(0.06, 0, 0.03);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(1);
    timer = new Timer();
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(desiredDegree);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(driveSubsystem.getHeading());
    output *= -1;
    output = output > 0.4 ? 0.4 : output;
    output = output < -0.4 ? -0.4 : output;
    driveSubsystem.drive(0, 0, output, true, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Finished", true);
    timer.stop();
    timer.stop();
    driveSubsystem.drive(0, 0, 0, true, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint() || timer.get() >= 2;
  }
}
