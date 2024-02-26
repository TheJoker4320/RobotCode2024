// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;;

public class Arm extends SubsystemBase {
	/** Creates a new Arm. */
	private final CANSparkMax ownerMotor;
	private final CANSparkMax SlaveMotor;
	private final AbsoluteEncoder encoder;
	private final PIDController pidController;
	private boolean constrain;
	private static Arm instance;

	public Arm() {
		// Initialize the Arm motor
		ownerMotor = new CANSparkMax(ArmConstants.MOTOR_ID2, ArmConstants.MOTOR_TYPE);
		SlaveMotor = new CANSparkMax(ArmConstants.MOTOR_ID1, ArmConstants.MOTOR_TYPE);

		ownerMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
		SlaveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
		SlaveMotor.follow(ownerMotor, true);
		ownerMotor.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);
		ownerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		constrain = true;
		// Initialize the Arm encoder
		encoder = ownerMotor.getAbsoluteEncoder(Type.kDutyCycle);
		encoder.setPositionConversionFactor(ArmConstants.CONVERT_RATE);
		encoder.setZeroOffset(ArmConstants.ENCODER_OFFSET);
		encoder.setInverted(true);
		
		// Initialize the PID controller for Arm pid control
		pidController = new PIDController(ArmConstants.PID_P,ArmConstants.PID_I ,ArmConstants.PID_D);
		pidController.setTolerance(1);
    }
    
    /**
     * the angle that the arm needs to be in in order to shoot to speaker
     * @param distance distance from robot to apriltag
     * @return the angle that the arm needs to be
     */
	public void swtichArmConstrain(){
		this.constrain = !this.constrain;
	}
	public void setSetpoint(double setpoint){
		pidController.setSetpoint(setpoint);
	}

    public static Arm getInstance(){
      if (instance == null)
        instance = new Arm();
      return instance;
    }

    public PIDController getCurrentPidController() {
        return pidController;
    }

    public double getAngleByDistanceSpeaker(double distance){
        distance = distance * 10;
        return -4.28 + 8.27E-3 * distance + 2.53E-5 * Math.pow(distance, 2) - 2.02E-8 * Math.pow(distance, 3) + 6.2E-12 * Math.pow(distance, 4) - 6.0E-16 * Math.pow(distance, 5);
    }

	public void setSpeed(double speed) {
		if(constrain){
			if(!((getPosition() > ArmConstants.MAX_DEGREES && speed > 0 && getPosition() < 350)
				|| ((getPosition() < ArmConstants.MIN_DEGREES || getPosition() > 350) && speed < 0 ))){ //software stop
					if((getPosition() < 30 && speed < 0) || (getPosition() > 70 && speed > 0)){ //Slowmode near edge
						ownerMotor.set(speed * ArmConstants.SLOW_SPEED);
					}
					else{
						ownerMotor.set(speed * ArmConstants.SPEED);
					}	
			}
			else{
				ownerMotor.set(0);
			}
		}
		else{
			ownerMotor.set(speed * ArmConstants.NO_CONSTRAIN_SPEED);
		}
		}

	public void setSpeedByMeasurement(double measurement){
		double output = pidController.calculate(measurement);
		output = output > 0.4 ? 0.4 : output;
		output = output < -0.4 ? -0.4 : output;
		ownerMotor.set(output);
	}

    // Stop the Arm motor by setting the speed to 0
    public void stop() {
        setSpeed(0);
    }

	public boolean atSetpoint(){
		return pidController.atSetpoint();
	}

    // Get the current position of the Arm motor
    public double getPosition() {
        return encoder.getPosition();
    }


	// Get the applied output of the Arm motor
	public double getAppliedOutput() {
		return ownerMotor.getAppliedOutput();
	}
	@Override
	public void periodic() {
	// This method will be called once per scheduler run
	SmartDashboard.putNumber("Arm encoder", encoder.getPosition());
	SmartDashboard.putBoolean("is constrain", constrain);
	}
}

