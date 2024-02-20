// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax masterMotor;
  private CANSparkMax slaveMotor;
  private RelativeEncoder encoder;
  private static Climber climberInstance;
  public Climber() {
    masterMotor = new CANSparkMax(11, MotorType.kBrushless);
    slaveMotor = new CANSparkMax(12, MotorType.kBrushless);
    masterMotor.setInverted(true);
    encoder = masterMotor.getEncoder();
    slaveMotor.follow(masterMotor, true);
  }
  public void elevate(double speed){
    masterMotor.set(speed);
  }
  public double getPosition(){
    return encoder.getPosition();
  }
  public static Climber getClimberInstance(){
    if (climberInstance == null)
      climberInstance = new Climber();
    return climberInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
