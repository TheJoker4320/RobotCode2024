// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.subsystems.Arm;

public class MoveToLLDegree extends Command {
  private Arm arm;
  private double output;
  private Timer timer;
  public MoveToLLDegree(Arm arm) {
    this.arm = arm;
    this.timer = new Timer();
    addRequirements(arm);
}
  
  @Override
  public void initialize() {
    timer.start();
    arm.setSetpoint(LimeLight.GetArmAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setSpeedByMeasurement(arm.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atSetpoint() || (arm.getPosition() < 3 && output < 0) || timer.get() > 4;
  }
}
