// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

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
    // Initialize the Arm motor
    OwnerMotor = new CANSparkMax(Constants.ArmConstants.MOTOR_ID2, Constants.ArmConstants.MOTOR_TYPE);
    SlaveMotor = new CANSparkMax(Constants.ArmConstants.MOTOR_ID1, Constants.ArmConstants.MOTOR_TYPE);
    OwnerMotor.restoreFactoryDefaults();
    SlaveMotor.restoreFactoryDefaults();
    SlaveMotor.follow(OwnerMotor, true);
    OwnerMotor.setSmartCurrentLimit(Constants.ArmConstants.CLAW_CURRENT_LIMIT);
    OwnerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Initialize the Arm encoder
    encoder = OwnerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(Constants.ArmConstants.CONVERT_RATE);
    encoder.setZeroOffset(Constants.ArmConstants.ENCODER_OFFSET + 2);
    encoder.setInverted(true);

    // Initialize the PID controller for Arm current control
    currentPid = OwnerMotor.getPIDController();
    currentPid.setFeedbackDevice(OwnerMotor.getEncoder());

    currentPid.setP(Constants.ArmConstants.CURRENTPID_P);
    currentPid.setI(Constants.ArmConstants.CURRENTPID_I);
    currentPid.setD(Constants.ArmConstants.CURRENTPID_D);
    setPidController(Constants.ArmConstants.CURRENT_PID);
    }
    
    /**
     * the angle that the arm needs to be in in order to shoot to speaker
     * @param distance distance from robot to apriltag
     * @return the angle that the arm needs to be
     */
    public double getArmAngle(double distance){
        return Constants.DistanceToAngle.m * distance + Constants.DistanceToAngle.constant;
    }
    public void reachArmPosition(double distance){
        double degreesToTarget = getArmAngle(distance);
        currentPid.setReference(degreesToTarget, ControlType.kPosition); 
    }
    public boolean isDegreesReached(double degrees){
       return Math.abs(encoder.getPosition() - degrees) < Constants.ArmConstants.CUREENTPID_TOLORANCE;
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

    // Stop the Arm motor by setting the speed to 0
    public void stop() {
        setSpeed(0);
    }

    // Get the Arm components
    
    // public void setIdleMode(IdleMode idleMode){
    //   OwnerMotor.setIdleMode(idleMode);
    //   SlaveMotor.setIdleMode(idleMode);
    // }

    // Set the PID controller gains for the Arm
    public void setPidController(PIDController terms) {
        getCurrentPidController().setP(terms.getP());
        getCurrentPidController().setI(terms.getI());
        getCurrentPidController().setD(terms.getD());
        SlaveMotor.getPIDController().setP(terms.getP());
        SlaveMotor.getPIDController().setI(terms.getI());
        SlaveMotor.getPIDController().setD(terms.getD());

    }

    // Set the current for the Arm motor
    // public void setCurrent(double current) {
    //     OwnerMotor.getPIDController().setReference(current, CANSparkMax.ControlType.kCurrent);
    //     OwnerMotor.getPIDController().setReference(current, CANSparkMax.ControlType.kCurrent);
    // }


    // Set the position for the Arm motor
    public void setMotorPosition(double distance) {
        getCurrentPidController().setReference(distance, CANSparkMax.ControlType.kPosition);
        SlaveMotor.getPIDController().setReference(distance, CANSparkMax.ControlType.kPosition);
    }

    // Check if the Arm motor is on target position
    public boolean isOnTarget(double distance) {
        return distance < getPosition();
    }

    // Get the current position of the Arm motor
    public double getPosition() {
        return encoder.getPosition();
    }

    // Get the current of the Arm motor
    public double getCurrent() {
        return OwnerMotor.getOutputCurrent();
    }

    // Get the temperature of the Arm motor
    public double getTemp() {
        return OwnerMotor.getMotorTemperature();
    }

    // Get the applied output of the Arm motor
    public double getAppliedOutput() {
        return OwnerMotor.getAppliedOutput();
    }
}

