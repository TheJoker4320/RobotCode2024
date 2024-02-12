// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int kCollectBtn = XboxController.Button.kA.value;
    public static final int kEjectBtn = XboxController.Button.kB.value;
  }

  public static final class CollectorConstants {
    public static final int COLLECTOR_PORT = 15;
    public static final int LIMIT_SWITCH_CHANNEL = 0;// TODO: correct the channel

    public static final double COLLECTOR_SPEED = 0.2;
  }
}
