// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Collect;
import frc.robot.commands.MoveToDegree;
import frc.robot.commands.MoveToKnownDegree;
import frc.robot.commands.MoveToLLDegree;
import frc.robot.commands.autonomousCommands.AimToTarget;
import frc.robot.commands.autonomousCommands.DriveByTime;
import frc.robot.commands.autonomousCommands.RotateDegrees;
import frc.robot.commands.autonomousCommands.ShootMaintainSpeed;
import frc.robot.commands.autonomousCommands.ShootReachSpeed;
import frc.robot.commands.autonomousCommands.Stay;
import frc.robot.commands.autonomousCommands.resetModuleOrientation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class AutoCreator 
{
    public Command getShootSequenceCommand(DriveSubsystem robotDrive, Shooter shooter, Collector collector, Arm arm)
    {
        return new SequentialCommandGroup(
          new AimToTarget(robotDrive),
          new ParallelDeadlineGroup(
            new MoveToLLDegree(arm),
            new ShootMaintainSpeed(shooter, 51, true)),
          new ParallelRaceGroup(
            new ShootMaintainSpeed(shooter,55.5, true),
            new Collect(collector, true)));
    }
    
    private TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
                ).setKinematics(DriveConstants.kDriveKinematics);

    //red command for mid note
    public Command redShootSpeakerTwiceAndLeave(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
    {
    Trajectory driveToCollect = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 5.53, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDrive = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(new Translation2d(2.33, 5.53)),
      new Pose2d(2.13, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.35, 5.53, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(6.39, 4.67, new Rotation2d(0)), 
      config.setReversed(false)); //TODO: Check This

    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollect,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDrive,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollect.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToLeave
    );
  }

  //blue command for mid note
  public Command blueShootSpeakerTwiceAndLeave(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollect = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 5.53, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDrive = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.35, 5.53, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(6.39, 6.39, new Rotation2d(0)), 
      config.setReversed(false));

    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollect,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDrive,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollect.getInitialPose());
    return new SequentialCommandGroup(
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToLeave
    );
  }

  //blue command for mid note with top note
  public Command getBlueShootSpeakerGoAmp(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 5.53, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));
    
    Trajectory driveToCollectSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 5.53, new Rotation2d(Math.PI / 2)),
      List.of(),
      new Pose2d(1.45, 7, new Rotation2d(Math.PI / 2)), 
      config.setReversed(false));
    Trajectory collectDriveSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 7, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.4, 7, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.4, 7, new Rotation2d(Math.PI / 2)),
      List.of(),
      new Pose2d(2.4, 7.62, new Rotation2d(Math.PI / 2)), 
      config.setReversed(false));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToCollectSecond = new SwerveControllerCommand(
      driveToCollectSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandCollectDriveSecond = new SwerveControllerCommand(
      collectDriveSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      new RotateDegrees(m_robotDrive, 90),
      commandDriveToCollectSecond,
      new RotateDegrees(m_robotDrive, 0),
      new ParallelDeadlineGroup(
        commandCollectDriveSecond,
        new Collect(collector, false)
      ),
      new RotateDegrees(m_robotDrive, 90),
      commandDriveToLeave
    );
  }

  //red command for mid note and top node
  public Command getRedShootSpeakerGoAmp(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 5.53, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));
    
    Trajectory driveToCollectSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 5.53, new Rotation2d(- Math.PI / 2)),
      List.of(),
      new Pose2d(1.45, 4.06, new Rotation2d(- Math.PI / 2)), 
      config.setReversed(true));
    Trajectory collectDriveSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 4.06, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.4, 4.06, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.4, 4.06, new Rotation2d(- Math.PI / 2)),
      List.of(),
      new Pose2d(2.4, 3.44, new Rotation2d(- Math.PI / 2)), 
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToCollectSecond = new SwerveControllerCommand(
      driveToCollectSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandCollectDriveSecond = new SwerveControllerCommand(
      collectDriveSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      new RotateDegrees(m_robotDrive, -90),
      commandDriveToCollectSecond,
      new RotateDegrees(m_robotDrive, 0),
      new ParallelDeadlineGroup(
        commandCollectDriveSecond,
        new Collect(collector, false)
      ),
      new RotateDegrees(m_robotDrive, -90),
      commandDriveToLeave
    );
  }

  //blue command to shoot starting note, collect top and leave
  public Command getBlueSpeakerTwiceLeaveAreaTop(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectTop = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 6.98, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveTop = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 6.98, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 6.98, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 6.98, new Rotation2d(- Math.PI / 2)),
      List.of(),
      new Pose2d(6.12, 6.98, new Rotation2d(- Math.PI / 2)), 
      config.setReversed(false));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectTop,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveTop,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectTop.getInitialPose());
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0)),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0)),
      commandDriveToLeave
    );
  }

  /**
   * @return Red command to shoot starting note, collect top note and leave WORKKSKSKKSKSKSKSK
  */
  public Command getRedSpeakerTwiceLeaveAreaTop(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectTop = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 6.98, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 6.98, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(6.12, 6.98, new Rotation2d(0)), 
      config.setReversed(false));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectTop,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectTop.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, 45),
      new WaitCommand(0.5),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      new RotateDegrees(m_robotDrive, 0),
      new WaitCommand(0.5),
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandDriveToCollect,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new RotateDegrees(m_robotDrive, 45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0)),
      commandDriveToLeave
    );
  }

    /**
   * @return Red command to shoot starting note, collect top note and shoot
  */
  public Command getRedSpeakerTwiceMid(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectTop = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 6.98, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveTop = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 6.98, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 6.98, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 6.98, new Rotation2d(0)),
      List.of(),
      new Pose2d(6.12, 6.98, new Rotation2d(0)), 
      config.setReversed(false));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectTop,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveTop,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectTop.getInitialPose());
    return new SequentialCommandGroup(
      //new RotateDegrees(m_robotDrive, 45),
      //new WaitCommand(0.5),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      //new MoveToDegree(arm, 0),
      //new RotateDegrees(m_robotDrive, 0),
      new WaitCommand(0.5),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.2),
      //new RotateDegrees(m_robotDrive, 45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm)
      //new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0))
      //commandDriveToLeave
    );
  }

  //blue command to shoot starting note, collect bottom and leave
  public Command getBlueSpeakerTwiceLeaveAreaBot(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectBot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 4.13, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 4.13, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveBot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 4.13, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 4.13, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 4.13, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 4.13, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 4.13, new Rotation2d(0)),
      List.of(new Translation2d(3.13, 1.6)),
      new Pose2d(6.12, 1.6, new Rotation2d(0)), 
      config.setReversed(false));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectBot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveBot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectBot.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0)),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0)),
      commandDriveToLeave
    );
  }

  //red command to shoot starting note, collect bottom and leave
  public Command getRedSpeakerTwiceLeaveAreaBot(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectBot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 4.13, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 4.13, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveBot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 4.13, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 4.13, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 4.13, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 4.13, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 4.13, new Rotation2d(0)),
      List.of(new Translation2d(3.13, 5.73)),
      new Pose2d(6.12, 5.73, new Rotation2d(0)), 
      config.setReversed(false));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectBot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveBot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectBot.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0)),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0).alongWith(new RotateDegrees(m_robotDrive, 0)),
      commandDriveToLeave
    );
  }

  //blue command to shoot starting note, collect bottom and leave
  public Command getBlueStealNote4FromMid(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 5.53, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));
    
    Trajectory driveToCollectSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 5.53, new Rotation2d(- Math.PI / 2)),
      List.of(),
      new Pose2d(1.45, 4.06, new Rotation2d(- Math.PI / 2)), 
      config.setReversed(true));
    Trajectory collectDriveSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 4.06, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.4, 4.06, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.4, 4.06, new Rotation2d(- Math.PI / 2)),
      List.of(),
      new Pose2d(2.4, 3.44, new Rotation2d(- Math.PI / 2)), 
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToCollectSecond = new SwerveControllerCommand(
      driveToCollectSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandCollectDriveSecond = new SwerveControllerCommand(
      collectDriveSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      new RotateDegrees(m_robotDrive, -90),
      commandDriveToCollectSecond,
      new RotateDegrees(m_robotDrive, 0),
      new ParallelDeadlineGroup(
        commandCollectDriveSecond,
        new Collect(collector, false)
      ),
      new RotateDegrees(m_robotDrive, -90),
      commandDriveToLeave
    );
  }

    //red command to shoot starting note, collect bottom and leave
  public Command getRedStealNote4(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.13, 5.53, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory collectDriveMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.13, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));
    
    Trajectory driveToCollectSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 5.53, new Rotation2d(- Math.PI / 2)),
      List.of(),
      new Pose2d(1.45, 4.06, new Rotation2d(- Math.PI / 2)), 
      config.setReversed(true));
    Trajectory collectDriveSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 4.06, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.4, 4.06, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory driveToLeave = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.4, 4.06, new Rotation2d(- Math.PI / 2)),
      List.of(),
      new Pose2d(2.4, 3.44, new Rotation2d(- Math.PI / 2)), 
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandCollectDrive = new SwerveControllerCommand(
      collectDriveMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToCollectSecond = new SwerveControllerCommand(
      driveToCollectSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandCollectDriveSecond = new SwerveControllerCommand(
      collectDriveSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToLeave = new SwerveControllerCommand(
      driveToLeave,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController,
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.5),
      commandDriveToShoot,
      new WaitCommand(0.5),
      new MoveToDegree(arm, 8),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      new RotateDegrees(m_robotDrive, -90),
      commandDriveToCollectSecond,
      new RotateDegrees(m_robotDrive, 0),
      new ParallelDeadlineGroup(
        commandCollectDriveSecond,
        new Collect(collector, false)
      ),
      new RotateDegrees(m_robotDrive, -90),
      commandDriveToLeave
    );
  }


  /** @return shoot note, collect top red or Bot Blue note and shoots it */
  public Command getTwoNoteTopRedOrBotBlue(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, 45),
      new WaitCommand(0.05),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new WaitCommand(0.05),
      new ParallelCommandGroup(
        new RotateDegrees(m_robotDrive, 0),
        new MoveToDegree(arm, 0.75)
      ),
      new ParallelDeadlineGroup(
        commandDriveToCollect,
        new Collect(collector, false)
      ),
      new WaitCommand(0.05),
      commandDriveToShoot,
      new WaitCommand(0.05),
      new RotateDegrees(m_robotDrive, 45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0.75)
    );
  }

    /** @return shoot note, collect Bottom Red or Top Blue note and shoots it */
  public Command getTwoNoteBotRedOrTopBlue(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));

    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, -45),
      new WaitCommand(0.05),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new WaitCommand(0.05),
      new RotateDegrees(m_robotDrive, 0),
      new ParallelDeadlineGroup(
        commandDriveToCollect,
        new MoveToDegree(arm, 0.75),
        new Collect(collector, false)
      ),
      new WaitCommand(0.05),
      commandDriveToShoot,
      new WaitCommand(0.05),
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0.75)
    );
  }

  /** @return shoot note, collect Middle Red or Middle Blue note and shoots it */
  public Command getTwoNoteMid(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.53, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.53, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      new WaitCommand(0.2),
      new ParallelDeadlineGroup(
        commandDriveToCollect,
        new Collect(collector, false)
      ),
      new WaitCommand(0.2),
      commandDriveToShoot,
      new WaitCommand(0.5),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0)
    );
  }
  /** @return shoots note and drives out of starting area. Driver needs to reset heading at teleop start */
  public Command getShootAndDriveOut(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm){
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new DriveByTime(2, 0.5, 0, m_robotDrive)
    );
  }

  public Command getWaitAndDrive(DriveSubsystem m_robotDrive, int time){
    return new WaitCommand(time).andThen(new DriveByTime(2, 0.5, 0, m_robotDrive)); 
  }

  /** @return Shoot first note, collect mid, shoot, collect shoot */
  public Command getShootRotate2CollectRight(DriveSubsystem m_robotDrive, Arm arm, Collector collector, Shooter shooter){
    Trajectory driveToCollectFirst = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.53, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToCollectSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.53, 5.53, new Rotation2d(Math.PI / 2)),
      List.of(),
      new Pose2d(2.53, 7.03, new Rotation2d(Math.PI / 2)), //FIXME: check correct distance
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollectFirst = new SwerveControllerCommand(
      driveToCollectFirst,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToCollectSecond = new SwerveControllerCommand(
      driveToCollectSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectFirst.getInitialPose());
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new ParallelDeadlineGroup(
        commandDriveToCollectFirst,
        new Collect(collector, false)
      ),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new RotateDegrees(m_robotDrive, 90),
      new ParallelDeadlineGroup(
        commandDriveToCollectSecond,
        new Collect(collector, false)),
      new RotateDegrees(m_robotDrive, 45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm)
    );
  }

    /** @return Shoot first note, collect mid, shoot, collect left, shoot */
  public Command getShootRotate2CollectLeft(DriveSubsystem m_robotDrive, Arm arm, Collector collector, Shooter shooter){
    Trajectory driveToCollectFirst = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.53, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToCollectSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.53, 5.53, new Rotation2d(-Math.PI / 2)),
      List.of(),
      new Pose2d(2.53, 7.03, new Rotation2d(-Math.PI / 2)), //FIXME: check correct distance
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollectFirst = new SwerveControllerCommand(
      driveToCollectFirst,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToCollectSecond = new SwerveControllerCommand(
      driveToCollectSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectFirst.getInitialPose());
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new ParallelDeadlineGroup(
        commandDriveToCollectFirst,
        new Collect(collector, false)
      ),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new RotateDegrees(m_robotDrive, -90),
      new ParallelDeadlineGroup(
        commandDriveToCollectSecond,
        new Collect(collector, false)),
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm)
    );
  }

  /** @return TEST */
  public Command getTest(DriveSubsystem m_robotDrive, Arm arm, Collector collector, Shooter shooter){
    Trajectory driveToCollectFirst = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.53, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToCollectSecond = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.53, 5.53, new Rotation2d(-Math.PI / 2)),
      List.of(),
      new Pose2d(2.53, 7.03, new Rotation2d(-Math.PI / 2)), //FIXME: check correct distance
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollectFirst = new SwerveControllerCommand(
      driveToCollectFirst,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToCollectSecond = new SwerveControllerCommand(
      driveToCollectSecond,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectFirst.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, 45),
      new WaitCommand(3),
      new RotateDegrees(m_robotDrive, 0)
    );
  }


      /** @return shoot note, collect & shoot mid, collect & shoot left */
  public Command getThreeNoteLeft(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToCollectLeft = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 5.53, new Rotation2d(Math.PI / 2)),
      List.of(),
      new Pose2d(2.60, 7.23, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory driveToShootLeft = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.60, 7.23, new Rotation2d(0)), 
      List.of(),
      new Pose2d(1.45, 5.75, new Rotation2d(Math.PI / 2)),
      config.setReversed(true));

    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToCollectLeft = new SwerveControllerCommand(
      driveToCollectLeft,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToShootLeft = new SwerveControllerCommand(
      driveToShootLeft,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new ParallelDeadlineGroup(
        new MoveToDegree(arm, 0.75),
        commandDriveToCollect,
        new Collect(collector, false)
      ),
      commandDriveToShoot,
      new WaitCommand(0.05),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new RotateDegrees(m_robotDrive, 0),
      new ParallelDeadlineGroup( 
        new SequentialCommandGroup(
          commandDriveToCollectLeft,
          new WaitCommand(0.05),
          commandDriveToShootLeft
        ),
        new MoveToDegree(arm, 0),
        new Collect(collector, false)
      ),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm)
    );
  }

  
      /** @return shoot note, collect & shoot mid, collect & shoot Right */
  public Command getThreeNoteRight(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.33, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.33, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.45, 5.53, new Rotation2d(0)), 
      config.setReversed(true));

    Trajectory driveToCollectRight = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.45, 5.53, new Rotation2d(Math.PI / 2)),
      List.of(),
      new Pose2d(2.60, 3.83, new Rotation2d(0)), 
      config.setReversed(false));

    Trajectory driveToShootRight = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.60, 3.83, new Rotation2d(0)), 
      List.of(),
      new Pose2d(1.45, 5.75, new Rotation2d(Math.PI / 2)),
      config.setReversed(true));

    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToCollectRight = new SwerveControllerCommand(
      driveToCollectRight,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    SwerveControllerCommand commandDriveToShootRight = new SwerveControllerCommand(
      driveToShootRight,
      m_robotDrive::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new ParallelDeadlineGroup(
        new MoveToDegree(arm, 0.75),
        commandDriveToCollect,
        new Collect(collector, false)
      ),
      commandDriveToShoot,
      new WaitCommand(0.05),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new RotateDegrees(m_robotDrive, 0),
      new ParallelDeadlineGroup( 
        new SequentialCommandGroup(
          commandDriveToCollectRight,
          new WaitCommand(0.05),
          commandDriveToShootRight
        ),
        new MoveToDegree(arm, 0),
        new Collect(collector, false)
      ),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm)
    );
  }
  public Command get2NoteBlueTopRedBotton(Shooter shooter, Collector collector, Arm arm, DriveSubsystem driveSubsystem)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.43, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.43, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.35, 5.53, new Rotation2d(0)), 
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      driveSubsystem::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      driveSubsystem::setModuleStates,
      driveSubsystem);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      driveSubsystem::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      driveSubsystem::setModuleStates,
      driveSubsystem);

    driveSubsystem.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(driveSubsystem, -42.5),
      new MoveToKnownDegree(arm),
      new SequentialCommandGroup(
        new ShootMaintainSpeed(shooter, 62, true),
        new ParallelRaceGroup(
          new ShootMaintainSpeed(shooter,65, true),
          new Collect(collector, true)
        )
      ),
      new RotateDegrees(driveSubsystem, 0),
      new MoveToDegree(arm, 0),
      new ParallelDeadlineGroup(
        commandDriveToCollect,
        new Collect(collector, false)
      ),
      new WaitCommand(0.1),
      commandDriveToShoot,
      new RotateDegrees(driveSubsystem, -41.5),
      new MoveToKnownDegree(arm),
      new SequentialCommandGroup(
        new ShootMaintainSpeed(shooter, 62, true),
        new ParallelRaceGroup(
          new ShootMaintainSpeed(shooter,65, true),
          new Collect(collector, true)
        )
      ),
      new MoveToDegree(arm, 0)
    );
  }

    public Command get2NoteBlueBottonRedTop(Shooter shooter, Collector collector, Arm arm, DriveSubsystem driveSubsystem)
  {
    Trajectory driveToCollectMid = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.35, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(2.43, 5.53, new Rotation2d(0)), 
      config.setReversed(false));
    
    Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.43, 5.53, new Rotation2d(0)),
      List.of(),
      new Pose2d(1.35, 5.53, new Rotation2d(0)), 
      config.setReversed(true));


    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xController = new PIDController(
      AutoConstants.kPXController,
      AutoConstants.kIXController,
      AutoConstants.kDXController);

    PIDController yController = new PIDController(
      AutoConstants.kPYController,
      AutoConstants.kIYController,
      AutoConstants.kDYController);
    
    SwerveControllerCommand commandDriveToCollect = new SwerveControllerCommand(
      driveToCollectMid,
      driveSubsystem::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      driveSubsystem::setModuleStates,
      driveSubsystem);
    
    SwerveControllerCommand commandDriveToShoot = new SwerveControllerCommand(
      driveToShoot,
      driveSubsystem::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController,
      driveSubsystem::setModuleStates,
      driveSubsystem);

    driveSubsystem.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(driveSubsystem, 42.5),
      new MoveToKnownDegree(arm),
      new SequentialCommandGroup(
        new ShootMaintainSpeed(shooter, 62, true),
        new ParallelRaceGroup(
          new ShootMaintainSpeed(shooter,65, true),
          new Collect(collector, true)
        )
      ),
      new RotateDegrees(driveSubsystem, 0),
      new MoveToDegree(arm, 0),
      new ParallelDeadlineGroup(
        commandDriveToCollect,
        new Collect(collector, false)
      ),
      new WaitCommand(0.1),
      commandDriveToShoot,
      new RotateDegrees(driveSubsystem, 41.5),
      new MoveToKnownDegree(arm),
      new SequentialCommandGroup(
        new ShootMaintainSpeed(shooter, 62, true),
        new ParallelRaceGroup(
          new ShootMaintainSpeed(shooter,65, true),
          new Collect(collector, true)
        )
      ),
      new MoveToDegree(arm, 0)
    );
  }
}