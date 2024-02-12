// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  /** Creates a new Collector. */
  private static Collector collectorInstance;
  private final TalonSRX m_main;
  private final DigitalInput m_limitSwitch;

  public Collector() {
    this.m_main = new TalonSRX(Constants.CollectorConstants.COLLECTOR_PORT);
    this.m_limitSwitch = new DigitalInput(Constants.CollectorConstants.LIMIT_SWITCH_CHANNEL);
  }

  public static Collector getInstance() {
    if (collectorInstance == null) {
      collectorInstance = new Collector();
    }
    return collectorInstance;
  }

  public boolean getLimitSwitch() {
    return m_limitSwitch.get();
  }

  public void setSpeed(double speed) {
    m_main.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}