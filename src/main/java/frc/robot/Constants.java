// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  // Deadband
  public static final double joystick_headband = 0.15;
  // Robot dimensions and speed
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(11.339);
  public static final double TRACK_WIDTH_X =
      Units.inchesToMeters(22.398); // X is perpendicular to the front of the robot
  public static final double TRACK_WIDTH_Y =
      Units.inchesToMeters(22.398); // Y is parallel to the front of the robot
  public static final double DRIVE_GEAR_RATIO = 60.0 / 9.0;
  public static final double TURN_GEAR_RATIO = 1.25 * 396.0 / 35.0; // (24.0 / 8) * (72.0 / 14);
  // Motor and encoder constants
  public static final int DRIVE_SPARK_MAX_FL = 24;
  public static final int TURN_SPARK_MAX_FL = 25;
  public static final int CANCODER_FL = 33;
  public static final double ABSOLUTE_ENCODER_OFFSET_FL = 0.668;
  public static final int DRIVE_SPARK_MAX_FR = 26;
  public static final int TURN_SPARK_MAX_FR = 27;
  public static final int CANCODER_FR = 30;
  public static final double ABSOLUTE_ENCODER_OFFSET_FR = 0.780 - 0.5;
  public static final int DRIVE_SPARK_MAX_BL = 23;
  public static final int TURN_SPARK_MAX_BL = 22;
  public static final int CANCODER_BL = 32;
  public static final double ABSOLUTE_ENCODER_OFFSET_BL = 0.054;
  public static final int DRIVE_SPARK_MAX_BR = 21;
  public static final int TURN_SPARK_MAX_BR = 20;
  public static final int CANCODER_BR = 31;
  public static final double ABSOLUTE_ENCODER_OFFSET_BR = 0.646 + 0.5;
  // Gyro
  public static final int PIGEON2_CAN_ID = 26;
  // Odometry
  public static final double ODOMETRY_FREQUENCY = 250.0; // ! was oficially 100.0
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.05) * 1 / 1.1132075472;
  public static final boolean FIELD_MIRROR = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
