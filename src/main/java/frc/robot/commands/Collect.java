// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class Collect extends Command {
  /** Creates a new Collect. */
  private Collector m_Collector;
  private Boolean m_isShoot;
  public Collect(Collector m_collector, boolean m_isShoot) {
    this.m_Collector = m_collector;
    this.m_isShoot = m_isShoot;
    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Collector.setCollectorState(true);
    if(!m_isShoot){
      LimeLight.setLedMode(2);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_Collector.setSpeed(CollectorConstants.COLLECTOR_SPEED);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Collector.setCollectorState(false);
    m_Collector.setSpeed(0);
    LimeLight.setLedMode(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_isShoot)
      return false;
    return m_Collector.getLimitSwitch();
  }
}