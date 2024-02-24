// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Collect;
import frc.robot.commands.ElevateByPlayer;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveToLLDegree;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.Shoot;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final DriveSubsystem m_robotDrive = DriveSubsystem.getInstance();
  private final Climber climber= Climber.getInstance();
  private final Collector collector = Collector.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Arm arm = Arm.getInstance();
  private final LimeLight limeLight = new LimeLight();
  private final AutoCreator autoCreator = new AutoCreator();
  private final SendableChooser<Command> m_chooser;
  private final PoseEstimatorUtils m_poseEstimator = new PoseEstimatorUtils(m_robotDrive, limeLight);
  // Creating the XboxController
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final PS4Controller m_operatorController = new PS4Controller(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Red side 2 notes from top side", autoCreator.getRedSpeakerTwiceLeaveAreaTop(shooter, collector, m_robotDrive, arm));
    m_chooser.addOption("Blue side 2 notes from top side", autoCreator.getBlueSpeakerTwiceLeaveAreaTop(shooter, collector, m_robotDrive, arm));

    SmartDashboard.putData(m_chooser);
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

    //R1
    JoystickButton shootSpeakerBtn = new JoystickButton(m_operatorController, OperatorConstants.kShootSpeakerBtn);
    shootSpeakerBtn.toggleOnTrue(new SequentialCommandGroup(new AimToTarget(m_robotDrive),
                            new ParallelDeadlineGroup(new MoveToLLDegree(arm),
                            new ShootReachSpeed(shooter, 60)),
                            new ParallelCommandGroup(new Stay(arm, false),
                            new SequentialCommandGroup(new ShootReachSpeed(shooter, 60),
                            new ParallelCommandGroup(new ShootMaintainSpeed(shooter, 60, false),
                            new Collect(collector, true))))));

    //R2
    JoystickButton raiseBtn = new JoystickButton(m_operatorController, OperatorConstants.kRaiseBtn);
    raiseBtn.whileTrue(new MoveArm(arm, false));

    //L2
    JoystickButton lowerBtn = new JoystickButton(m_operatorController, OperatorConstants.kLowerBtn);
    lowerBtn.whileTrue(new MoveArm(arm, true));

    //SHARE
    JoystickButton stayBtn = new JoystickButton(m_operatorController, OperatorConstants.kStayBtn);
    stayBtn.toggleOnTrue(new Stay(arm, false));
    
    //CIRCLE
    JoystickButton shootAmpBtn = new JoystickButton(m_operatorController, OperatorConstants.kshootAmpBtn);
    shootAmpBtn.whileTrue(new ParallelCommandGroup(new ShootMaintainSpeed(shooter, 20, false),new Collect(collector, true)));
    //PS
    JoystickButton switchArmConstrainBtn = new JoystickButton(m_operatorController, OperatorConstants.kSwitchArmConstrainBtn);
    switchArmConstrainBtn.onTrue(new SwitchArmConstrain(arm));
    
    //Touchpad
    JoystickButton shootManualBtn = new JoystickButton(m_operatorController, OperatorConstants.kShootManualBtn);
    shootManualBtn.toggleOnTrue(new SequentialCommandGroup(new ShootReachSpeed(shooter, 60), new ParallelCommandGroup(new ShootMaintainSpeed(shooter, 60, false),
    new Collect(collector, true))));

    //L3
     JoystickButton prepareShooterBtn = new JoystickButton(m_operatorController, OperatorConstants.kPrepareShooterBtn);
      prepareShooterBtn.whileTrue(new Shoot(shooter, 1));
      //R3
      JoystickButton collectToShootBtn = new JoystickButton(m_operatorController, OperatorConstants.kCollectToShootBtn);
      collectToShootBtn.whileTrue(new Collect(collector, true));
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

  public void zeroHeadingNonSpecific()
  {
    m_robotDrive.zeroHeading();
  }

  
  /**
   * Use this to pass the autonomous Pcommand to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    return m_chooser.getSelected();
  }
}
