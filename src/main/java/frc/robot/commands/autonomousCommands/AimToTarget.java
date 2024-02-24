// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import org.opencv.core.TickMeter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.subsystems.DriveSubsystem;

public class AimToTarget extends Command {
  private final LimeLight limelight;
  private final PIDController pidController;
  private final DriveSubsystem driveSubsystem;
  private double lastMeasure;
  private Timer timer;
  public AimToTarget(final DriveSubsystem driveSubsystem,final LimeLight limelight) {

    this.limelight = limelight;
    pidController = new PIDController(Constants.LimeLightConstants.AIMING_KP, Constants.LimeLightConstants.AIMING_KI, Constants.LimeLightConstants.AIMING_KD);
    this.driveSubsystem = driveSubsystem;
    pidController.setTolerance(1);
    this.timer = new Timer();
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(0);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotValue;
    if(limelight.doesLimeLightHaveTargets())
      lastMeasure = limelight.getLimeLightXValue();
    rotValue = pidController.calculate(lastMeasure);
    driveSubsystem.drive(0,0,rotValue,true,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setX();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if(limelight.getLimeLightXValue() < 5 && limelight.getLimeLightXValue() >= -5)
      //return true;
    return pidController.atSetpoint() || timer.get() >= 3;
  }
}
