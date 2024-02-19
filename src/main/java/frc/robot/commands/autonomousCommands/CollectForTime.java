// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class CollectForTime extends Command {
  private final Collector collector;
  private final int time;

  private final Timer timer;
  
  /**
   * This constructor creates a command of collect for time
   * this command collects for a set time at a set output value
   * 
   * @param collector The collector subsystem
   * @param time The time which the collector will collect for
   */
  public CollectForTime(Collector collector, int time) 
  {
    this.collector = collector;
    this.time = time;

    timer = new Timer();
    addRequirements(collector);
  }

  /**
   * This method will be called once at the start of the command
   * this function will start the timer
   */
  @Override
  public void initialize() 
  {
    timer.start();
  }

  /**
   * This method is called once per schedueler run, it will set the collectors output
   * to a given value in the CollectorConstants class
   */
  @Override
  public void execute() 
  {
    collector.setSpeed(CollectorConstants.COLLECTOR_SPEED);
    SmartDashboard.putNumber("collector timer", timer.get());
  }

  /**
   * This method will be called once the command is either stopped or interrupted
   * This method will set the collector's speed to 0
   * 
   * @param interrupted Whether this command was interrupted or stopped naturaly
   */
  public void end(boolean interrupted) 
  {
    timer.stop();
    timer.reset();
    collector.setSpeed(0);
  }

  /**
   * This method returns true when the command should
   * stop. Meaning the moment the timer ran out.
   * @return Whether this command should end or not
   */
  @Override
  public boolean isFinished() {
    return (timer.get() >= time);
  }
}
