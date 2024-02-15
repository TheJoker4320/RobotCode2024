// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final CANSparkMax OwnerMotor;
  private final CANSparkMax SlaveMotor;
  private final AbsoluteEncoder encoder;
  private final SparkPIDController currentPid;
  private static Arm instance;
  public Arm() {
    // Initialize the claw motor
    OwnerMotor = new CANSparkMax(Constants.ClawConstants.MOTOR_ID2, Constants.ClawConstants.MOTOR_TYPE);
    SlaveMotor = new CANSparkMax(Constants.ClawConstants.MOTOR_ID1, Constants.ClawConstants.MOTOR_TYPE);
    OwnerMotor.restoreFactoryDefaults();
    SlaveMotor.restoreFactoryDefaults();
    SlaveMotor.follow(OwnerMotor, true);
    OwnerMotor.setSmartCurrentLimit(Constants.ClawConstants.CLAW_CURRENT_LIMIT);
    OwnerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Initialize the claw encoder
    encoder = OwnerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(Constants.ClawConstants.CONVERT_RATE);
    encoder.setZeroOffset(Constants.ClawConstants.ENCODER_OFFSET + 2);
    encoder.setInverted(true);

    // Initialize the PID controller for claw current control
    currentPid = OwnerMotor.getPIDController();
    currentPid.setFeedbackDevice(OwnerMotor.getEncoder());
    setPidController(Constants.ClawConstants.CURRENT_PID);
    }

    public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder", encoder.getPosition());
    }

    public static Arm getInstance(){
      if (instance == null)
        instance = new Arm();
      return instance;
    }

    public SparkPIDController getCurrentPidController() {
        return currentPid;
    }

    public void setSpeed(double speed) {
        OwnerMotor.set(speed);
    }

    // Stop the claw motor by setting the speed to 0
    public void stop() {
        setSpeed(0);
    }

    // Get the claw components
    
    // public void setIdleMode(IdleMode idleMode){
    //   OwnerMotor.setIdleMode(idleMode);
    //   SlaveMotor.setIdleMode(idleMode);
    // }

    // Set the PID controller gains for the claw
    public void setPidController(PIDController terms) {
        getCurrentPidController().setP(terms.getP());
        getCurrentPidController().setI(terms.getI());
        getCurrentPidController().setD(terms.getD());
        SlaveMotor.getPIDController().setP(terms.getP());
        SlaveMotor.getPIDController().setI(terms.getI());
        SlaveMotor.getPIDController().setD(terms.getD());

    }

    // Set the current for the claw motor
    // public void setCurrent(double current) {
    //     OwnerMotor.getPIDController().setReference(current, CANSparkMax.ControlType.kCurrent);
    //     OwnerMotor.getPIDController().setReference(current, CANSparkMax.ControlType.kCurrent);
    // }


    // Set the position for the claw motor
    public void setMotorPosition(double distance) {
        getCurrentPidController().setReference(distance, CANSparkMax.ControlType.kPosition);
        SlaveMotor.getPIDController().setReference(distance, CANSparkMax.ControlType.kPosition);
    }

    // Check if the claw motor is on target position
    public boolean isOnTarget(double distance) {
        return distance < getPosition();
    }

    // Get the current position of the claw motor
    public double getPosition() {
        return encoder.getPosition();
    }

    // Get the current of the claw motor
    public double getCurrent() {
        return OwnerMotor.getOutputCurrent();
    }

    // Get the temperature of the claw motor
    public double getTemp() {
        return OwnerMotor.getMotorTemperature();
    }

    // Get the applied output of the claw motor
    public double getAppliedOutput() {
        return OwnerMotor.getAppliedOutput();
    }
}

