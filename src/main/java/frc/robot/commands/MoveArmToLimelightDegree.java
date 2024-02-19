// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.subsystems.Arm;

public class MoveArmToLimelightDegree extends Command {
  private Arm arm;
  private LimeLight limeLight;
  private PIDController pidController;
  public MoveArmToLimelightDegree(Arm arm, LimeLight limeLight) {
    this.arm = arm;
    this.limeLight = limeLight;
    pidController = new PIDController(0.1, 0, 0);
    pidController.setTolerance(0.5);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(arm.getAngleByDistanceSpeaker(limeLight.getTrueDistance()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(arm.getPosition());
    output = output > 0.2 ? 0.2 : output;
    output = output < -0.2 ? -0.2 : output;
    arm.setSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
