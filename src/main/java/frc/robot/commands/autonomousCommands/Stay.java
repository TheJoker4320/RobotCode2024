// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Stay extends Command {
  private final Arm arm;
  private double desiredAngle;
  private PIDController pidController;
  public Stay(Arm arm) {
    this.arm = arm;
    pidController = new PIDController(0.03, 0, 0);
    addRequirements(arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = arm.getPosition();
    pidController.setSetpoint(arm.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ArmPosition = arm.getPosition();
    double output = pidController.calculate(ArmPosition);
    output = output > 0.1 ? 0.1 : output;
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
    return false;
  }
}