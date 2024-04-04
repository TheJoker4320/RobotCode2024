// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveToDegree extends Command {
  private Arm arm;
  private PIDController pidController;
  private double degree;
  private double output;
  private Timer timer;
  public MoveToDegree(Arm arm, double degree) {
    this.arm = arm;
    this.degree = degree;
    pidController = new PIDController(0.1, 0, 0);
    pidController.setTolerance(0.75);
    timer = new Timer();
    addRequirements(arm);
}
  
  @Override
  public void initialize() {
    pidController.setSetpoint(degree);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = pidController.calculate(arm.getPosition());
    output = 0.4 < output ? 0.5 : output;
    output = -0.4 > output ? -0.5 : output;
    arm.setSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint() || (arm.getPosition() < 3 && output < 0) || (arm.getPosition() > 90 && output > 0) || timer.get() >= 2;
  }
}
