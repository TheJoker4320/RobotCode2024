package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule 
{
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final SparkPIDController m_turningPIDController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   * 
   * @param drivingCANId The can id of the driving motor
   * @param turningCANId The can id of the turning motor
   * @param chassisAngularOffset The angular offset of this module from the chassis
   * @param drivingInvereted Wether the driving motor is inverted or not
   */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean drivingInverted)
    {
        m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        m_drivingSparkMax.setInverted(drivingInverted);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();

        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,ModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        m_turningPIDController.setP(ModuleConstants.kTurningP);
        m_turningPIDController.setI(ModuleConstants.kTurningI);
        m_turningPIDController.setD(ModuleConstants.kTurningD);
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,ModuleConstants.kTurningMaxOutput); 

        // Set the idle modes and their smart current limiters
        m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }
}
