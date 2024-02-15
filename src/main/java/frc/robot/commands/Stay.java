// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Stay extends Command {
  private final PIDController CURRENT_PID;
  private final Arm arm;

  public Stay(Arm arm) {
    this.arm = arm;
    CURRENT_PID = new PIDController(6, 0, 0);
    addRequirements(arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CURRENT_PID.setSetpoint(arm.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ArmPosition = arm.getPosition();
    double output = CURRENT_PID.calculate(ArmPosition);
    arm.setSpeed(output);
    SmartDashboard.putNumber("output", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
