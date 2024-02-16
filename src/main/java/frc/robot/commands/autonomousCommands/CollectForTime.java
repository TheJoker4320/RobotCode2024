// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class CollectForTime extends Command {
  private final Collector collector;
  private final int time;

  private final Timer timer;
  
  /** Creates a new CollectForTime. */
  public CollectForTime(Collector collector, int time) 
  {
    this.collector = collector;
    this.time = time;

    timer = new Timer();

    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    collector.setSpeed(CollectorConstants.COLLECTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    collector.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= time);
  }
}
