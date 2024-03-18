// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase 
{
  private static boolean isFieldRelative = true;
  private static DriveSubsystem driveSubsystem;
  private double inputMultiplier = 1;
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset,
    false,
    DriveConstants.kFrontLeftModuleId);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset,
    false,
    DriveConstants.kFrontRightModuleId);
    
  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset,
    false,
    DriveConstants.kRearLeftModuleId);
      
  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset,
    false,
    DriveConstants.kRearRightModuleId);
  
  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private final StructArrayPublisher<SwerveModuleState> publisher;

  // Variables for keeping track of current angle assuming the starting angle isnt 0
  private double m_realStartingAngle;
  private double m_currentRealAngle;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(0),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  
  /** Creates a new DriveSubsystem. */
  private DriveSubsystem() 
  {
    //Zeroes heading.
    //TODO: This constructor might require changing since its not guranteed we zero the angle here.
    zeroHeading();
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    m_realStartingAngle = 0;

    isFieldRelative = true;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-1 * getCurrentRealAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    publisher.set(new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    });
    SmartDashboard.putNumber("Robot heading", -1 * getCurrentRealAngle());
    SmartDashboard.putNumber("Speed Multiplier", inputMultiplier);
    SmartDashboard.putBoolean ("IsFieldRelative", isFieldRelative);
  }

  /**
   * This methods sets the angle actual start, this is adviced to be used in cases where the robot doesnt start in the
   * angle 0, so we can tell him what his current angle actually is
   * 
   * @param realStartAngle What the real starting angle
   */
  public void setRealStartAngle(double realStartAngle)
  {
    m_realStartingAngle = realStartAngle;
  }
  
  /** Zeroes the heading of the robot.
   * it also resets the real starting angle, should be used only if
   * you know the robot is at the angle 0 at that exact moment
   */
  public void zeroHeading() {
    m_gyro.zeroYaw();
    m_realStartingAngle = 0;
    m_currentRealAngle = 0;
  }

  /** Zeroes the heading of the robot.
   * this should be used when you know the robot is not at the 0 angle
   * this method should be later paired with the method:
   * setRealStartngle(realStartAngle)
   */
  public void zeroHeadingNonSpecific()
  {
    m_gyro.zeroYaw();
  }

  public double getCurrentRealAngle()
  {
    if (Math.abs(m_realStartingAngle) < 0.15)
      return m_gyro.getYaw();
    else
    {
      double currentGyroAngle = m_gyro.getYaw();
      double actualAngle = m_realStartingAngle + currentGyroAngle;
      actualAngle = actualAngle > 180 ? actualAngle - 360 : actualAngle;
      return actualAngle;
    } 
  }

  public void setInputMultiplier(double inputMultiplier){
    this.inputMultiplier = inputMultiplier;
  }
  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() 
  {
    return m_odometry.getPoseMeters();
  }
  public SwerveModulePosition[] getModulePosition(){
    SwerveModulePosition[] swerveModuleStates =  {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};
    return swerveModuleStates;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  public static DriveSubsystem getInstance(){
    if(driveSubsystem == null){
      driveSubsystem = new DriveSubsystem();
    }

    return driveSubsystem;
  }

  public void resetOdometry(Pose2d pose) 
  {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw()),
        new SwerveModulePosition[] 
        {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void changeFieldRelative()
  {
    this.isFieldRelative = !this.isFieldRelative;
  }
  
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean robotfieldRelative, boolean rateLimit, boolean shouldOverride) 
  {
    boolean fieldRelative;
    if (shouldOverride)
      fieldRelative = robotfieldRelative; //Argument
    else
      fieldRelative = isFieldRelative; //Member
    
    xSpeed = xSpeed * inputMultiplier;
    ySpeed = ySpeed * inputMultiplier;
    rot = rot * inputMultiplier;

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) 
    {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));


      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) 
      {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } 
      else 
      {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

      if (angleDif < 0.45*Math.PI)
      {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI)
      {
        if (m_currentTranslationMag > 1e-4) 
        { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else 
        {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else 
      {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);
    } 
    else 
    {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;


    SwerveModuleState[] swerveModuleStates;
    if (fieldRelative)
    {
      double xSpeedsAdjusted = xSpeedDelivered * Math.cos(Units.degreesToRadians(getCurrentRealAngle() * -1)) + ySpeedDelivered *  Math.sin(Units.degreesToRadians(getCurrentRealAngle() * -1));
      double ySpeedsAdjusted = -1 * xSpeedDelivered * Math.sin(Units.degreesToRadians(getCurrentRealAngle() * -1)) + ySpeedDelivered *  Math.cos(Units.degreesToRadians(getCurrentRealAngle() * -1));
      

      
      swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      new ChassisSpeeds(xSpeedsAdjusted,
                        ySpeedsAdjusted,
                        rotDelivered));
    }
    else
    {
      swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    }
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   * Meaning that all the modules are "looking" at each other which prevents movement
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Sets the swerve ChassisSpeeds
   * 
   * @param desiredChassisSpeeds The desired Chassis speeds
   */
  public void setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds)
  {
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the current ChassisSpeeds of the swerve
   * 
   * @return Current ChassisSpeeds of the swerve
   */
  public ChassisSpeeds getChassisSpeeds()
  {
    SwerveModuleState[] currentStates = new SwerveModuleState[4];
    currentStates[0] = m_frontLeft.getState();
    currentStates[1] = m_frontRight.getState();
    currentStates[2] = m_rearLeft.getState();
    currentStates[3] = m_rearRight.getState();

    return DriveConstants.kDriveKinematics.toChassisSpeeds(currentStates);
  }

  public void setModulesDirection(Double angle)
  {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    desiredStates[0] = new SwerveModuleState(0, new Rotation2d(angle));
    desiredStates[1] = new SwerveModuleState(0, new Rotation2d(angle));
    desiredStates[2] = new SwerveModuleState(0, new Rotation2d(angle));
    desiredStates[3] = new SwerveModuleState(0, new Rotation2d(angle));

    setModuleStates(desiredStates);
  }

}
