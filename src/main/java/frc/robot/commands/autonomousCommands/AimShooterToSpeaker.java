// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLight;
import frc.robot.subsystems.Arm;

public class AimShooterToSpeaker extends Command {
  /** Creates a new AimShooterToSpeaker. */
  private Arm arm;
  private LimeLight limelight; //TODO: calculate distance with pose estimator
  public AimShooterToSpeaker(Arm arm, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.limelight = limelight;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.reachArmPosition(limelight.getTrueDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isDegreesReached(arm.getArmAngle(limelight.getTrueDistance()));
  }
}
