package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;

public class Claw extends SubsystemBase {
    // Abstract class for claw components - motors, encoder, and PID controller
    public interface ClawComponents {
        CANSparkMax getOwnerMotor();
        CANSparkMax getSlaveMotor();
        AbsoluteEncoder getClawEncoder();
        SparkPIDController getCurrentPidController();
    }
    
    private final ClawComponents components;

    public Claw(ClawComponents components) {
        this.components = components;
    }

    private static Claw instance;

    // Set the speed of the claw motor
    public void setSpeed(double speed) {
        components.getOwnerMotor().set(speed);
    }

    // Stop the claw motor by setting the speed to 0
    public void stop() {
        setSpeed(0);
    }

    // Get the claw components
    public ClawComponents getComponents() {
        return components;
    }

    // Set the PID controller gains for the claw
    public void setPidController(PIDController terms) {
        components.getCurrentPidController().setP(terms.getP());
        components.getCurrentPidController().setI(terms.getI());
        components.getCurrentPidController().setD(terms.getD());
    }

    // Set the current for the claw motor
    public void setCurrent(double current) {
        components.getCurrentPidController().setReference(current, CANSparkMax.ControlType.kCurrent, 0);
    }

    // Set the position for the claw motor
    public void setMotorPosition(double distance) {
        components.getCurrentPidController().setReference(distance, CANSparkMax.ControlType.kPosition);
    }

    // Check if the claw motor is on target position
    public boolean isOnTarget(double distance) {
        return distance < getPosition();
    }

    // Get the current position of the claw motor
    public double getPosition() {
        return components.getClawEncoder().getPosition();
    }

    // Get the current of the claw motor
    public double getCurrent() {
        return components.getOwnerMotor().getOutputCurrent();
    }

    // Get the temperature of the claw motor
    public double getTemp() {
        return components.getOwnerMotor().getMotorTemperature();
    }

    // Get the applied output of the claw motor
    public double getAppliedOutput() {
        return components.getOwnerMotor().getAppliedOutput();
    }

    // Initialize the Claw instance with the provided ClawComponents
    public static void init(ClawComponents components) {
        instance = new Claw(components);
    }

    // Get the Claw instance
    public static Claw getInstance() {
        return instance;
    }

    // Implementation of the ClawComponents interface
    public class ClawComponentsImpl implements ClawComponents {

        private final CANSparkMax OwnerMotor;
        private final CANSparkMax SlaveMotor;
        private final AbsoluteEncoder encoder;
        private final SparkPIDController currentPid;

        public ClawComponentsImpl() {
            // Initialize the claw motor
            OwnerMotor = new CANSparkMax(Constants.ClawConstants.MOTOR_ID, Constants.ClawConstants.MOTOR_TYPE);
            SlaveMotor = new CANSparkMax(Constants.ClawConstants.MOTOR_ID, Constants.ClawConstants.MOTOR_TYPE);
            OwnerMotor.setInverted(true);
            SlaveMotor.follow(OwnerMotor);
            OwnerMotor.restoreFactoryDefaults();
            OwnerMotor.setSmartCurrentLimit(Constants.ClawConstants.CLAW_CURRENT_LIMIT);
            OwnerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            OwnerMotor.setInverted(true);

            // Initialize the claw encoder
            encoder = OwnerMotor.getAbsoluteEncoder(Type.kDutyCycle);
            encoder.setPositionConversionFactor(Constants.ClawConstants.CONVERT_RATE);

            // Initialize the PID controller for claw current control
            currentPid = OwnerMotor.getPIDController();
            currentPid.setFeedbackDevice(OwnerMotor.getEncoder());
            currentPid.setP(Constants.ClawConstants.CURRENT_PID.getP());
            currentPid.setI(Constants.ClawConstants.CURRENT_PID.getI());
            currentPid.setD(Constants.ClawConstants.CURRENT_PID.getD());
            currentPid.setIZone(Constants.ClawConstants.I_ZONE);
            currentPid.setOutputRange(Constants.ClawConstants.MIN_VALUE, Constants.ClawConstants.MAX_VALUE);
        }

        @Override
        public CANSparkMax getOwnerMotor() {
            return OwnerMotor;
        }
        @Override
        public CANSparkMax getSlaveMotor() {
            return SlaveMotor;
        }

        @Override
        public AbsoluteEncoder getClawEncoder() {
            return encoder;
        }

        @Override
        public SparkPIDController getCurrentPidController() {
            return currentPid;
        }
    }
}
