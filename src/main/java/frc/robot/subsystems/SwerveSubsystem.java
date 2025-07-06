// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

  public final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kIsFrontRightDriveEncoderReversed,
    DriveConstants.kIsFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.DriveAbsoluteEncoderOffsetRad.kBackRight,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

public final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kIsFrontLeftDriveEncoderReversed,
    DriveConstants.kIsFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.DriveAbsoluteEncoderOffsetRad.kFrontLeft,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

public final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kIsBackRightDriveEncoderReversed,
    DriveConstants.kIsBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.DriveAbsoluteEncoderOffsetRad.kBackRight,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

public final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kIsBackLeftDriveEncoderReversed,
    DriveConstants.kIsBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.DriveAbsoluteEncoderOffsetRad.kBackLeft,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

private final AHRS gyro = new AHRS(AHRS.NavXComType.kUSB1);

public SwerveSubsystem() {
  new Thread(() -> {
    try {
      Thread.sleep(1000);
      zeroHeading();
    } catch (Exception e) {

    }
  }).start();
}


public void zeroHeading(){
  gyro.reset();
}


public double getHeading(){
  return Math.IEEEremainder(gyro.getAngle(), 360);
}

public Rotation2d getRotation2d(){
  return Rotation2d.fromDegrees(getHeading());
}

@Override
public void periodic(){
  SmartDashboard.putNumber("Robot Heading", getHeading());
}

public void stopModules(){
  frontLeft.stop();
  frontRight.stop();
  backLeft.stop();
  backRight.stop();
}

public void setModuleStates(SwerveModuleState[] desiredStates){
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  frontLeft.setDesiredState(desiredStates[0]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[3]);
}


}

