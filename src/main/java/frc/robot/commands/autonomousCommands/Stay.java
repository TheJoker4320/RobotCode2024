// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Stay extends Command {
  private final Arm arm;
  private final Timer timer;
  private final boolean isTimer;
  public Stay(Arm arm, boolean isTimer) {
    this.arm = arm;
    this.timer = new Timer();
    this.isTimer = isTimer;
    addRequirements(arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isTimer)
      timer.start();
    arm.setSetpoint(arm.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setSpeedByMeasurement(arm.getPosition());
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
    return false || timer.get() > 0.5;
  }
}
