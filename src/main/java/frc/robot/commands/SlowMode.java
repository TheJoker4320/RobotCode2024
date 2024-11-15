// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SlowMode extends InstantCommand {
  /** Creates a new SlowMode. */
  private DriveSubsystem driveSubsystem;
  private double inputMultiplier;
  public SlowMode(DriveSubsystem driveSubsystem, double inputMultiplier) {
    this.driveSubsystem = driveSubsystem;
    this.inputMultiplier = inputMultiplier;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setInputMultiplier(inputMultiplier);
  }
}
