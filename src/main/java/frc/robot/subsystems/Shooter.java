// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonSRX master;
  private TalonSRX slave;
  private Encoder encoder;

  private PIDController pidController = new PIDController(Constants.ShooterConstants.kP, Constants.ShooterConstants.kI,
      Constants.ShooterConstants.kD);
  private static Shooter shooter;

  public Shooter() {
    this.master = new TalonSRX(Constants.ShooterConstants.SHOOTER_MASTER_PORT);
    this.slave = new TalonSRX(Constants.ShooterConstants.SHOOTER_SLAVE_PORT);
    this.encoder = new Encoder(Constants.ShooterConstants.SHOOTER_ENCODER_PORT_A,
        Constants.ShooterConstants.SHOOTER_ENCODER_PORT_B);
    encoder.setDistancePerPulse(Constants.ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
    slave.follow(master);
  }

  public static Shooter getInstance(){
    if(shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }

  public void setOutput(double output)
  {
    master.set(TalonSRXControlMode.PercentOutput, output);
  }

  public void shoot(double speed){
   master.set(TalonSRXControlMode.PercentOutput, pidController.calculate(getSpeed(), speed));
  }

  public double getSpeed() {
    SmartDashboard.putNumber("Rate of shooter", getSpeed());
   return encoder.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
