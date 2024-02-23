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
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
	/** Creates a new Arm. */
	private final CANSparkMax OwnerMotor;
	private final CANSparkMax SlaveMotor;
	private final AbsoluteEncoder encoder;
	private final SparkPIDController pidController;
	private static Arm instance;

	public Arm() {
		// Initialize the Arm motor
		OwnerMotor = new CANSparkMax(ArmConstants.MOTOR_ID2, ArmConstants.MOTOR_TYPE);
		SlaveMotor = new CANSparkMax(ArmConstants.MOTOR_ID1, ArmConstants.MOTOR_TYPE);
		SlaveMotor.follow(OwnerMotor, true);
		OwnerMotor.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);
		OwnerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		// Initialize the Arm encoder
		encoder = OwnerMotor.getAbsoluteEncoder(Type.kDutyCycle);
		encoder.setPositionConversionFactor(ArmConstants.CONVERT_RATE);
		encoder.setZeroOffset(ArmConstants.ENCODER_OFFSET);
		encoder.setInverted(true);

<<<<<<< HEAD
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
=======
		// Initialize the PID controller for Arm current control
		pidController = OwnerMotor.getPIDController();
		pidController.setFeedbackDevice(encoder);
		
		pidController.setP(ArmConstants.CURRENTPID_P);
		pidController.setI(ArmConstants.CURRENTPID_I);
		pidController.setD(ArmConstants.CURRENTPID_D);
		pidController.setSmartMotionAllowedClosedLoopError(ArmConstants.CUREENTPID_TOLORANCE, 0);
		//setPidController(ArmConstants.CURRENT_PID);
	}
>>>>>>> origin/development

	public static Arm getInstance() {
		if (instance == null)
			instance = new Arm();
		return instance;
	}

	public boolean atSetpoint(double desiredAngle) {
		return desiredAngle <= getPosition() + pidController.getSmartMotionAllowedClosedLoopError(0)
				&& desiredAngle >= getPosition() - pidController.getSmartMotionAllowedClosedLoopError(0);
	}

	public void setSetpoint(double setPoint) {
		pidController.setReference(setPoint, ControlType.kPosition);
	}

<<<<<<< HEAD
    public double getAngleByDistanceSpeaker(double distance){
        distance = distance * 10;
        return -4.28 + 8.27E-3 * distance + 2.53E-5 * Math.pow(distance, 2) - 2.02E-8 * Math.pow(distance, 3) + 6.2E-12 * Math.pow(distance, 4) - 6.0E-16 * Math.pow(distance, 5);
    }

    public void setSpeed(double speed) {
        OwnerMotor.set(speed);
    }
=======
	public SparkPIDController getPidController() {
		return pidController;
	}
>>>>>>> origin/development

	public void setSpeed(double speed) {
		OwnerMotor.set(speed);
	}

	// Stop the Arm motor by setting the speed to 0
	public void stop() {
		setSpeed(0);
	}

	// Set the PID controller gains for the Arm
	public void setPidController(PIDController terms) {
		getPidController().setP(terms.getP());
		getPidController().setI(terms.getI());
		getPidController().setD(terms.getD());

		// TODO: check if it's possible to set PID only for master motor
		// SlaveMotor.getPIDController().setP(terms.getP());
		// SlaveMotor.getPIDController().setI(terms.getI());
		// SlaveMotor.getPIDController().setD(terms.getD());

	}
	// Set the position for the Arm motor
	// public void setMotorPosition(double distance) {
	// getPidController().setReference(distance, ControlType.kPosition);

	// // TODO: check if it's possible to setreference only for master motor
	// //SlaveMotor.getPIDController().setReference(distance,
	// ControlType.kPosition);
	// }

	// Get the current position of the Arm motor
	public double getPosition() {
		return encoder.getPosition();
	}

	// Get the current of the Arm motor
	public double getCurrent() {
		return OwnerMotor.getOutputCurrent();
	}

<<<<<<< HEAD
    // Get the current position of the Arm motor
    public double getPosition() {
        return encoder.getPosition() > 350 ? 0 : encoder.getPosition();
    }
=======
	// Get the temperature of the Arm motor
	public double getTemp() {
		return OwnerMotor.getMotorTemperature();
	}
>>>>>>> origin/development

	// Get the applied output of the Arm motor
	public double getAppliedOutput() {
		return OwnerMotor.getAppliedOutput();
	}

	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("encoder", encoder.getPosition());
	}
}

