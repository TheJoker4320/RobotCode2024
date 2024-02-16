// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShootForTime extends Command {
  private final Shooter shooter;
  private final int time;

  private final Timer timer;
  
  /**
   * This constructor creates a command of shoot for time
   * this command shoots for a set time at a set output value
   * 
   * @param shooter The shooter subsystem
   * @param time The time which the shooter will shoot for
   */
  public ShootForTime(Shooter shooter, int time) 
  {
    this.shooter = shooter;
    this.time = time;

    timer = new Timer();
    addRequirements(shooter);
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
   * This method is called once per schedueler run, it will set the shooter output
   * to a given value in the ShooterConstants class
   */
  @Override
  public void execute() 
  {
    shooter.setOutput(ShooterConstants.SHOOT_SPEED);
    SmartDashboard.putNumber("shooter timer", timer.get());
  }

  /**
   * This method will be called once the command is either stopped or interrupted
   * Pls read further notes in the method.
   * 
   * @param interrupted Whether this command was interrupted or stopped naturaly
   */
  @Override
  public void end(boolean interrupted) 
  {
    timer.stop();
    timer.reset();
    // TODO: In certain situations the end function should not set speed to 0, and instead do nothing.
    // TODO: maybe needed to have the command recieve wether to 0 at the end or not
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
