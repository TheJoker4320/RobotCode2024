// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.subsystems.Arm;

public class AimArmToTag extends Command {
  /** Creates a new AimArmToTag. */
  private LimeLight limelight;
  private Arm arm;
  private PIDController pidcontroller;
  public AimArmToTag(Arm arm, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.limelight = limelight;
    addRequirements(arm);
    pidcontroller = new PIDController(0, 0, 0);
    pidcontroller.setTolerance(0.5);
    pidcontroller.setSetpoint(limelight.getLimeLightYValue());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidcontroller.calculate(limelight.getLimeLightYValue());
    arm.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pidcontroller.getPositionTolerance()) > Math.abs(pidcontroller.getPositionError());
  }
}
