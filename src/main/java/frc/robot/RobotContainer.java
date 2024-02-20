// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants.OperatorConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AimToTarget;
import frc.robot.commands.Collect;
import frc.robot.commands.Eject;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmToLimelightDegree;
import frc.robot.commands.MoveToDegree;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.Shoot;
import frc.robot.commands.Stay;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

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
  private final Collector collector = Collector.getInstance();
  private final Shooter shooter = Shooter.GetInstance();
  private final Arm arm = Arm.getInstance();
  private final LimeLight limeLight = new LimeLight();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final AimToTarget aimToTarget = new AimToTarget(m_robotDrive, limeLight);
  private final Shoot shoot = new Shoot(shooter);
  private final Stay stay = new Stay(arm);
  // Creating the XboxController
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final PS4Controller m_operatorController = new PS4Controller(OperatorConstants.kOperatorControllerPort);

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
                -MathUtil.applyDeadband(m_driverController.getLeftY() * 1, OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * 1, OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * 1, OperatorConstants.kDriveDeadband),
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
    // JoystickButton resetHeadingBtn = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    // resetHeadingBtn.whileTrue(new ResetHeading(m_robotDrive));
    // JoystickButton collectBtn = new JoystickButton(m_driverController, OperatorConstants.kCollectBtn);
    // collectBtn.toggleOnTrue(new Collect(collector, false));
    // JoystickButton ejectBtn = new JoystickButton(m_driverController, OperatorConstants.kEjectBtn);
    // ejectBtn.whileTrue(new Eject(collector));
    // JoystickButton shooterBtn = new JoystickButton(m_driverController, OperatorConstants.kShootBtn);
    // shooterBtn.whileTrue(new Shoot(shooter));
    // JoystickButton collectToShoot = new JoystickButton(m_driverController, OperatorConstants.kCollectToShootBtn);
    // collectToShoot.whileTrue(new Collect(collector,true));
    // JoystickButton RaiseButton = new JoystickButton(m_operatorController, PS4Controller.Button.kR1.value);
    // RaiseButton.whileTrue(new MoveArm(arm, false));
    // JoystickButton LowerButton = new JoystickButton(m_operatorController, PS4Controller.Button.kL1.value);
    // LowerButton.whileTrue(new MoveArm(arm, true));
    // JoystickButton MoveToDegreeBtn = new JoystickButton(m_operatorController, PS4Controller.Button.kCircle.value);
    // MoveToDegreeBtn.toggleOnTrue(new MoveToDegree(arm, 35).andThen(new Stay(arm)));
    JoystickButton btnAim = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    btnAim.onTrue(new MoveArmToLimelightDegree(arm, limeLight));
    JoystickButton StayBtn = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    StayBtn.whileTrue(stay);
    
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
   * Use this to pass the autonomous Pcommand to the main {@link Robot} class.
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
