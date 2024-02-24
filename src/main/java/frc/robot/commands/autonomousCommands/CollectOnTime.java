// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class CollectOnTime extends Command {
  /** Creates a new Collect. */
  private Collector m_Collector;
  private Boolean m_isShoot;
  private Timer timer;
  private double timeout;

  public CollectOnTime(Collector m_collector, boolean m_isShoot, double timeout) {
    this.m_Collector = m_collector;
    this.m_isShoot = m_isShoot;
    this.timeout = timeout;

    this.timer = new Timer();

    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_Collector.setSpeed(CollectorConstants.COLLECTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Collector.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > timeout)
      return true;
    return false;
  }
}