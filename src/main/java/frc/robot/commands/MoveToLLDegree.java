// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.subsystems.Arm;

public class MoveToLLDegree extends Command {
  private Arm arm;
  private PIDController pidController;
  private double output;
  public MoveToLLDegree(Arm arm) {
    this.arm = arm;
    pidController = new PIDController(0.1, 0, 0);
    addRequirements(arm);
}
  
  @Override
  public void initialize() {
    pidController.setSetpoint(LimeLight.GetArmAngle());
    SmartDashboard.putNumber("Desired Angle", LimeLight.GetArmAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = pidController.calculate(arm.getPosition());
    output = 0.1 < output ? 0.1 : output;
    output = -0.1 > output ? -0.1 : output;
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
    return arm.getPosition() > LimeLight.GetArmAngle() || (arm.getPosition() < 3 && output < 0);
  }
}
