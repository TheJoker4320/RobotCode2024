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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Collect;
import frc.robot.commands.MoveToDegree;
import frc.robot.commands.MoveToLLDegree;
import frc.robot.commands.autonomousCommands.AimToTarget;
import frc.robot.commands.autonomousCommands.RotateDegrees;
import frc.robot.commands.autonomousCommands.ShootMaintainSpeed;
import frc.robot.commands.autonomousCommands.ShootReachSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class AutoCreator 
{
    private Command getShootSequenceCommand(DriveSubsystem robotDrive, Shooter shooter, Collector collector, Arm arm)
    { 
        return new SequentialCommandGroup(new AimToTarget(robotDrive),

                            new AimToTarget(robotDrive),
                            new MoveToLLDegree(arm),
                            new ShootReachSpeed(shooter, 60),
                            new ParallelRaceGroup(new ShootMaintainSpeed(shooter,60, true),
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
      new RotateDegrees(m_robotDrive, 45),
      new WaitCommand(0.5),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0),
      new WaitCommand(0.5),
      new RotateDegrees(m_robotDrive, 0),
      commandDriveToCollect,
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        commandCollectDrive,
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

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new RotateDegrees(m_robotDrive, 0),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.15),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.15),
      commandDriveToShoot,
      new WaitCommand(0.15),
      new RotateDegrees(m_robotDrive, -45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0)
    );
  }

    /** @return shoot note, collect Bottom Red or Top Blue note and shoots it */
  public Command getTwoNoteBotRedOrTopBlue(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
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

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      new RotateDegrees(m_robotDrive, 45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new RotateDegrees(m_robotDrive, 0),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.15),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.15),
      commandDriveToShoot,
      new WaitCommand(0.15),
      new RotateDegrees(m_robotDrive, 45),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0)
    );
  }

  /** @return shoot note, collect Middle Red or Middle Blue note and shoots it */
  public Command getTwoNoteMid(Shooter shooter, Collector collector, DriveSubsystem m_robotDrive, Arm arm)
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

    m_robotDrive.resetOdometry(driveToCollectMid.getInitialPose());
    return new SequentialCommandGroup(
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new RotateDegrees(m_robotDrive, 0),
      new MoveToDegree(arm, 0),
      commandDriveToCollect,
      new WaitCommand(0.15),
      new ParallelDeadlineGroup(
        commandCollectDrive,
        new Collect(collector, false)
      ),
      new WaitCommand(0.15),
      commandDriveToShoot,
      new WaitCommand(0.15),
      getShootSequenceCommand(m_robotDrive, shooter, collector, arm),
      new MoveToDegree(arm, 0)
    );
  }
}
