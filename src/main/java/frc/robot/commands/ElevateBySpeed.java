// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ElevateBySpeed extends Command {
  private Climber climber;
  private PIDController pidController;
  public ElevateBySpeed() {
    climber = Climber.getInstance();
    pidController = new PIDController(0, 0, 0);
    pidController.setTolerance(0.5);
    pidController.setSetpoint(50);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(climber.getPosition());
    climber.elevate(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.elevate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.getPositionError() <= pidController.getPositionTolerance();
  }
}
