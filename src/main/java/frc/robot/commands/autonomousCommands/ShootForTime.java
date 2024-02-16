// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShootForTime extends Command {
  private final Shooter shooter;
  private final int time;

  private final Timer timer;
  
  /** Creates a new ShootForTime. */
  public ShootForTime(Shooter shooter, int time) 
  {
    this.shooter = shooter;
    this.time = time;

    timer = new Timer();
    
    addRequirements(shooter);
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
    shooter.shoot(ShooterConstants.SHOOT_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // TODO: In certain situations the end function should not set speed to 0, and instead do nothing.
    // TODO: maybe needed to have the command recieve wether to 0 at the end or not
    shooter.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= time);
  }
}
