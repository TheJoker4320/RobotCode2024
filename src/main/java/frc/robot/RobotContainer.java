// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Collect;
import frc.robot.commands.Eject;
import frc.robot.commands.ElevateByPlayer;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveToLLDegree;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.SlowMode;
import frc.robot.commands.SwitchArmConstrain;
import frc.robot.commands.autonomousCommands.AimToTarget;
import frc.robot.commands.autonomousCommands.ShootMaintainSpeed;
import frc.robot.commands.autonomousCommands.ShootReachSpeed;
import frc.robot.commands.autonomousCommands.Stay;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.utils.PoseEstimatorUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autonomousCommands.StraightPidDrive;
import frc.robot.commands.autonomousCommands.resetModuleOrientation;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Climber climber= Climber.getInstance();
  private final Collector collector = Collector.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Arm arm = Arm.getInstance();
  private final LimeLight limeLight = new LimeLight();
  private final PoseEstimatorUtils m_poseEstimator = new PoseEstimatorUtils(m_robotDrive, limeLight);
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
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
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
    ///////////////////Driver///////////////////
    //LB
    JoystickButton resetHeadingBtn = new JoystickButton(m_driverController, OperatorConstants.kZeroHeadingBtn);
    resetHeadingBtn.whileTrue(new ResetHeading(m_robotDrive));

    //A
    JoystickButton slowSpeedBtn = new JoystickButton(m_driverController, OperatorConstants.kSlowBtn);
    slowSpeedBtn.onTrue(new SlowMode(m_robotDrive, 0.3));

    //X
    JoystickButton moderateSpeedBtn = new JoystickButton(m_driverController, OperatorConstants.kModerateBtn);
    moderateSpeedBtn.onTrue(new SlowMode(m_robotDrive, 0.7));

    //Y
    JoystickButton normalSpeedBtn = new JoystickButton(m_driverController, OperatorConstants.kNormalBtn);
    normalSpeedBtn.onTrue(new SlowMode(m_robotDrive, 1));

    ///////////////////Operator///////////////////
    //OPTIONS
    JoystickButton climbBtn = new JoystickButton(m_operatorController, OperatorConstants.kClimbBtn);
    climbBtn.whileTrue(new ElevateByPlayer(climber));

    //CROSS
    JoystickButton collectBtn = new JoystickButton(m_operatorController, OperatorConstants.kCollectBtn);
    collectBtn.toggleOnTrue(new Collect(collector, false));

    //SQUARE
    JoystickButton ejectBtn = new JoystickButton(m_operatorController, OperatorConstants.kEjectBtn);
    ejectBtn.whileTrue(new Eject(collector));

    //R1
    JoystickButton shootSpeakerBtn = new JoystickButton(m_operatorController, OperatorConstants.kShootSpeakerBtn);
    shootSpeakerBtn.toggleOnTrue(new SequentialCommandGroup(new AimToTarget(m_robotDrive, limeLight),
                            new ParallelDeadlineGroup(new MoveToLLDegree(arm),
                            new ShootReachSpeed(shooter, 60)),
                            new ParallelCommandGroup(new Stay(arm),
                            new SequentialCommandGroup(new ShootReachSpeed(shooter, 60),
                            new ParallelCommandGroup(new ShootMaintainSpeed(shooter,60),
                            new Collect(collector, true))))));

    //R2
    JoystickButton raiseBtn = new JoystickButton(m_operatorController, OperatorConstants.kRaiseBtn);
    raiseBtn.whileTrue(new MoveArm(arm, false));

    //L2
    JoystickButton lowerBtn = new JoystickButton(m_operatorController, OperatorConstants.kLowerBtn);
    lowerBtn.whileTrue(new MoveArm(arm, true));

    //SHARE
    JoystickButton stayBtn = new JoystickButton(m_operatorController, OperatorConstants.kStayBtn);
    stayBtn.toggleOnTrue(new Stay(arm));
    
    //CIRCLE
    JoystickButton shootAmpBtn = new JoystickButton(m_operatorController, OperatorConstants.kshootAmpBtn);
    shootAmpBtn.whileTrue(new ParallelCommandGroup(new ShootMaintainSpeed(shooter,60),
    new Collect(collector, true)));
    
    //PS
    JoystickButton switchArmConstrainBtn = new JoystickButton(m_operatorController, OperatorConstants.kSwitchArmConstrainBtn);
    switchArmConstrainBtn.onTrue(new SwitchArmConstrain(arm));
    

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
    m_robotDrive.resetEncoders();

    /* 
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

    List<Translation2d> bezierPoints2 = PathPlannerPath.bezierFromPoses(
      new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    );

    //This will create the path that the robot will follow using the constrants we give to him
    PathPlannerPath path2 = new PathPlannerPath(
      bezierPoints2, 
      new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
        AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
      new GoalEndState(0, Rotation2d.fromDegrees(0))
    );

    FollowPathHolonomic firstPath = new FollowPathHolonomic(
      path,
      m_robotDrive::getPose,
      m_robotDrive::getChassisSpeeds,
      m_robotDrive::setChassisSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
        new PIDConstants(AutoConstants.kPThetaController),
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kSwerveDriveRadius,
        new ReplanningConfig()
      ),
      this::getCurrentAlliance,
      m_robotDrive
    );

    FollowPathHolonomic secondPath = new FollowPathHolonomic(
      path2,
      m_robotDrive::getPose,
      m_robotDrive::getChassisSpeeds,
      m_robotDrive::setChassisSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
        new PIDConstants(AutoConstants.kPThetaController),
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kSwerveDriveRadius,
        new ReplanningConfig()
      ),
      this::getCurrentAlliance,
      m_robotDrive
    );

    return (new resetModuleOrientation(m_robotDrive)).andThen(new WaitCommand(1)).andThen(firstPath).andThen((new WaitCommand(2)).alongWith(new resetModuleOrientation(m_robotDrive))).andThen(secondPath);*/
    
    //-----------------------
    //-----------------------

    PIDController xPidController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    PIDController yPidController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController);
    PIDController thetaPidController = new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);

    return new ParallelRaceGroup(new WaitCommand(15), (
           new StraightPidDrive(m_robotDrive, xPidController, yPidController, thetaPidController, new Pose2d(2, 0, new Rotation2d(0)), 1)).andThen(
           new WaitCommand(1)).andThen(
           new StraightPidDrive(m_robotDrive, xPidController, yPidController, thetaPidController, new Pose2d(0, 0, new Rotation2d(0)), 2))
           );

    //-----------------------
    //-----------------------
  }
}
