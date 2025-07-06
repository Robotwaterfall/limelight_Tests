// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    
        // Drive and turning gear ratios.
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
    
        // Conversion factors for drive motor's position and velocity.
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    
        // Conversion factors for turn motor's position and velocity.
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    
        // The propertional coefficent for the turning PID controller.
        public static final double kTurningControllerPValue = 0.25;
    
      }

        public static final class DriveConstants {

    
    public static final double kTrackWidth = Units.inchesToMeters(19.75); 

    public static final double kWheelBase = Units.inchesToMeters(26.5); 
   
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    
    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kBackLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 7;
    public static final int kBackRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 10;
    public static final int kBackLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 4;


    public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 21;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 19;
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    public static final boolean kIsFrontLeftTurningEncoderReversed = true;
    public static final boolean kIsBackLeftTurningEncoderReversed = true;
    public static final boolean kIsFrontRightTurningEncoderReversed = true;
    public static final boolean kIsBackRightTurningEncoderReversed = true;

    public static final boolean kIsFrontLeftDriveEncoderReversed = false;
    public static final boolean kIsBackLeftDriveEncoderReversed = false;
    public static final boolean kIsFrontRightDriveEncoderReversed = true;
    public static final boolean kIsBackRightDriveEncoderReversed = true;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    
    public static final class DriveAbsoluteEncoderOffsetRad {
      public static final double kFrontLeft = 0.35864 * 2 * Math.PI;
      public static final double kBackLeft = 0.23388 * 2 * Math.PI;
      public static final double kFrontRight = -0.2861 * 2 * Math.PI;
      public static final double kBackRight = 0.08715 * 2 * Math.PI;
    }

    
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

     static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 3;
    
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;

    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static class autoTargetConstants {
     
      public static final double autoOrientKp = 0.0035;

    }
  }

  public final static class OIConstants {

    //prevents small changes in joystick from moving the robot
    public static final double kDeadband = 0.5;

    public static final int kDriverJoystickPort = 0;

    public static final int kDriverXAxis = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverRotAxis = 4;

    /** Configure robot into field oriented mode button. */
    public static final int kDriverFieldOrientedButtonIdx = 1;
  }

  
}
