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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Constants.MAX_LINEAR_SPEED;
  public static final double odometryFrequency = Constants.ODOMETRY_FREQUENCY; // Hz
  public static final double trackWidth = Constants.TRACK_WIDTH_X;
  public static final double wheelBase = Constants.TRACK_WIDTH_Y;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int pigeonCanId = Constants.PIGEON2_CAN_ID;

  public static final int frontLeftDriveCanId = Constants.DRIVE_SPARK_MAX_FL;
  public static final int backLeftDriveCanId = Constants.DRIVE_SPARK_MAX_BL;
  public static final int frontRightDriveCanId = Constants.DRIVE_SPARK_MAX_FR;
  public static final int backRightDriveCanId = Constants.DRIVE_SPARK_MAX_BR;

  public static final int frontLeftTurnCanId = Constants.TURN_SPARK_MAX_FL;
  public static final int backLeftTurnCanId = Constants.TURN_SPARK_MAX_BL;
  public static final int frontRightTurnCanId = Constants.TURN_SPARK_MAX_FR;
  public static final int backRightTurnCanId = Constants.TURN_SPARK_MAX_BR;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Constants.WHEEL_RADIUS;
  public static final double driveMotorReduction = Constants.DRIVE_GEAR_RATIO;
  public static final DCMotor driveGearbox = DCMotor.getNeo550(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 30;
  public static final double turnMotorReduction = Constants.TURN_GEAR_RATIO;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 0.1;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = Units.lbsToKilograms(Constants.ROBOT_MASS_LBS);
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
