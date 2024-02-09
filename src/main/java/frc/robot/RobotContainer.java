// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimToTarget;
import frc.robot.commands.DriveToTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorUtils;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PoseEstimatorUtils m_poseEstimator = new PoseEstimatorUtils();
  private final LimeLight limeLight = new LimeLight();
  // Creating the XboxController
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final AimToTarget aimToTarget = new AimToTarget(m_robotDrive, limeLight);
  private final DriveToTarget driveToTarget = new DriveToTarget(m_robotDrive, m_poseEstimator, limeLight);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * 0.5, OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.5, OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * 0.5, OperatorConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    JoystickButton btnAimToTarget = new JoystickButton(m_driverController, 2);
    btnAimToTarget.onTrue(aimToTarget);
    
    JoystickButton btnDriveToTarget = new JoystickButton(m_driverController, 3);
    btnDriveToTarget.onTrue(driveToTarget);
  }

  /**
   * Returns the current alliance as a boolean,
   * True reprsents the red alliance
   * False represents either not present or blue
   * 
   * @return Wether current alliance is red
   */
  private boolean getCurrentAlliance()
  {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      return alliance.get() == Alliance.Red;
    return false;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //This will hold the points on which the robots will need to go through.
    //If im (YONY) not wrong the rotation 2d value is the angle in which the robot should get to that position
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(1, 0, Rotation2d.fromDegrees(0))
    );

    //This will create the path that the robot will follow using the constrants we give to him
    PathPlannerPath path = new PathPlannerPath(
      bezierPoints, 
      new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
        AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
      new GoalEndState(0, Rotation2d.fromDegrees(0))
    );

    return new FollowPathHolonomic(
      path,
      m_robotDrive::getPose,
      m_robotDrive::getChassisSpeeds,
      m_robotDrive::setChassisSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(AutoConstants.kPXController),
        new PIDConstants(AutoConstants.kPThetaController),
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kSwerveDriveRadius,
        new ReplanningConfig()
      ),
      this::getCurrentAlliance,
      m_robotDrive
    );
  }
}
