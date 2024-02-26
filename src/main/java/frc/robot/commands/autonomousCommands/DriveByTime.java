// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveByTime extends Command {
  /** Creates a new DriveByTime. */
  private Timer timer;
  private double timeout;

  private double xAxisSpeed;
  private double yAxisSpeed;

  private DriveSubsystem driveSubsystem;

  public DriveByTime(double timeout, double xAxisSpeed, double yAxisSpeed, DriveSubsystem driveSubsystem) {
    this.timeout = timeout;
    this.xAxisSpeed = xAxisSpeed;
    this.yAxisSpeed = yAxisSpeed;

    this.timer = new Timer();

    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    driveSubsystem.drive(xAxisSpeed, yAxisSpeed, 0, false, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    timer.stop();
    timer.reset();

    driveSubsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= timeout;
  }
}
