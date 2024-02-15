// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveToDegree extends Command {
  private Arm arm;
  private PIDController pidController;
  public MoveToDegree(Arm arm, boolean isReversed) {
    this.arm = arm;
    pidController = new PIDController(0.2, 0, 0);
    addRequirements(arm);
}
  
  @Override
  public void initialize() {
    pidController.setSetpoint(Constants.ClawConstants.DESIRED_DEGREE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(arm.getPosition());
    arm.setSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getPosition() > Constants.ClawConstants.DESIRED_DEGREE;
  }
}
