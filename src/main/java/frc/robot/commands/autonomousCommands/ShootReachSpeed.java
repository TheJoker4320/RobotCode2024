// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootReachSpeed extends Command {
  /** Creates a new ShootReachSpeed. */
  private Shooter shooter;
  private double speed;
  private Timer timeout;

  public ShootReachSpeed(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeout.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shoot(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      timeout.stop();
      timeout.reset();
    }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return shooter.getSpeed() >= speed || timeout.get() >= 2;
    

    }

}
