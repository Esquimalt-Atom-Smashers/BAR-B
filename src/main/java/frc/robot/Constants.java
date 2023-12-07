// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.swerve.SecondOrderSwerveKinematics;
import frc.lib.swerve.SwerveModuleConstants;
import java.util.Arrays;
import java.util.List;
import java.util.TreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean competitionMode = false;

  public static final class GlobalConstants {
      //public static final String CANIVORE_NAME = "CANivore";
      public static final int PCM_ID = 19;
      public static final double targetVoltage = 12.0; // Used for voltage compensation

      public static final double batteryVoltageThreshold = 12.3;

      //public static final double minimumPressure = 100; // PSI
      //public static final double maximumPressure = 120; // try 120
  }

  public static final class ControllerConstants {
      public static final int DRIVE_CONTROLLER = 0;
      public static final int OPERATOR_CONTROLLER = 1;
  }


  public static final class SwerveConstants extends CompBotConstants {
    
  }

  public static class CompBotConstants {
      // See https://github.com/Team364/BaseFalconSwerve for getting these values.

      //public static final boolean hasPigeon = true;
      //ublic static final int PIGEON_PORT = 29;

      public static final double lengthWithBumpers = Units.inchesToMeters(32 + 3.25 * 2);
      public static final double widthWithBumpers = Units.inchesToMeters(27.5 + 3.25 * 2);

      public static final double trackWidth = Units.inchesToMeters(21.5);
      public static final double wheelBase = Units.inchesToMeters(26.75);
      public static final double wheelDiameter = Units.inchesToMeters(4.0);
      public static final double wheelCircumference = wheelDiameter * Math.PI;

      public static final double robotMass = Units.lbsToKilograms(45);

      public static final double openLoopRamp = 0.25; // 0.25
      public static final double closedLoopRamp = 0.0;

      public static final double driveGearRatio = (6.75 / 1.0); // 5.14:1
      public static final double angleGearRatio = (150/7 / 1.0); // 12.8:1

      public static final Translation2d[] moduleTranslations = new Translation2d[] {
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

      public static final SecondOrderSwerveKinematics swerveKinematics =
              new SecondOrderSwerveKinematics(moduleTranslations);

      /* Swerve Current Limiting */
      public static final int angleContinuousCurrentLimit = 25;
      public static final int anglePeakCurrentLimit = 40;
      public static final double anglePeakCurrentDuration = 0.1;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveContinuousCurrentLimit = 35;
      public static final int drivePeakCurrentLimit = 60;
      public static final double drivePeakCurrentDuration = 0.1;
      public static final boolean driveEnableCurrentLimit = true;

      /* Angle Motor PID Values */
      public static final double angleKP = 0.6;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;

      /* Drive Motor PID Values */
      public static final double driveKP = 0.1;
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;

      /* Motor Information */
      public static final double driveMotorFreeSpeed = 6380; // RPM of Falcon 500
      public static final double angleMotorFreeSpeed = 6380; // RPM of Falcon 500
      public static final double stallTorque = 4.69;

      /* Drive Motor Characterization Values */
      public static final double driveKS =
              (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
      public static final double driveKV = (2.44 / 12);
      public static final double driveKA = (0.27 / 12);

      /* Angle Motor Characterization Values */
      public static final double angleKS = 0;
      // (0.368 / 12); // divide by 12 to convert from volts to percent output for CTRE
      public static final double angleKV = (0.234 / 12);
      public static final double angleKA = (0.003 / 12);

      /* Swerve Profiling Values */
      public static final double maxSpeed = 6.52; // meters per second
      public static final double maxAcceleration =
              (stallTorque * driveGearRatio * 4) / (wheelDiameter * robotMass); // 16.52; // meters per second^2
      public static final double maxAngularVelocity = 5; // rad/s
        //       / Arrays.stream(moduleTranslations)
        //               .map(translation -> translation.getNorm())
        //               .max(Double::compare)
        //               .get();

      /* Calculated Characterization Values */
      public static final double calculatedDriveKS = 0;
      public static final double calculatedDriveKV = (12 / maxSpeed) / GlobalConstants.targetVoltage;
      public static final double calculatedDriveKA = (12 / maxAcceleration) / GlobalConstants.targetVoltage;
      public static final double calculatedAngleKV =
              (12 * 60) / (angleMotorFreeSpeed * Math.toRadians(360 / angleGearRatio));

      /* Precise Driving Mode Values */
      public static final double preciseDrivingModeSpeedMultiplier = 0.2;

      /* Neutral Modes */
      public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

      /* Drive Motor Inverts */
      public static final boolean driveMotorInvert = true;

      /* Drive Encoder Inverts */
      public static final boolean driveEncoderInvert = false;

      /* Angle Motor Inverts */
      public static final boolean angleMotorInvert = false;

      /* Angle Encoder Invert */
      public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.Clockwise_Positive;

      /* Module Specific Constants */
      // Note, bevel gears should face left (relative to back-to-front)

      /* Front Left Module - Module 0 */
      public static final class Mod0 {
          public static final int driveMotorID = 1;
          public static final int angleMotorID = 0;
          public static final int canCoderID = 0;
          public static final double angleOffset = .1582 * 360; 
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 {

          public static final int driveMotorID = 5;
          public static final int angleMotorID = 4;
          public static final int canCoderID = 2;
          public static final double angleOffset = .9797 * 360;
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Left Module - Module 2 */
      public static final class Mod2 {
          public static final int driveMotorID = 3;
          public static final int angleMotorID = 2;
          public static final int canCoderID = 1;
          public static final double angleOffset = .6382 * 360;
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 {
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 6;
          public static final int canCoderID = 3;
          public static final double angleOffset = .667 * 360;
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  }
}
