// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonSRX master;
  private TalonSRX slave;
  private static Shooter shooter;

  public Shooter() {
    this.master = new TalonSRX(Constants.ShooterConstants.SHOOTER_MASTER_PORT);
    this.slave = new TalonSRX(Constants.ShooterConstants.SHOOTER_SLAVE_PORT);    
    slave.follow(master);
  }

  public static Shooter GetInstance(){
    if(shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }

  //Set Shooter motor speed
  public void Shoot(double speed){
    master.set(TalonSRXControlMode.PercentOutput,speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
