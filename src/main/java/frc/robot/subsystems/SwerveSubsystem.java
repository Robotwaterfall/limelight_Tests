// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;
import java.lang.module.Configuration;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
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

private final SwerveModule swerveModules[] = new SwerveModule[]{
  frontLeft,frontRight,
  backLeft, backRight
};

 private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
    new Rotation2d(0), getModulePositionsAuto() );

private RobotConfig config;

public SwerveSubsystem() {
  new Thread(() -> {
    try {
      Thread.sleep(1000);
      zeroHeading();
    } catch (Exception e) {

    }
  }).start();

try{
            config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
    }
        
          AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

}

public SwerveModulePosition[] getModulePositionsAuto() { // not updating
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getSwerveModulePosition();
    }
    return positions;
  }



public void zeroHeading(){
  gyro.reset();
}

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    } 
    public void resetPose(Pose2d pose){
        odometer.resetPosition(gyro.getRotation2d(), getModulePositionsAuto() , pose);
    }
     public ChassisSpeeds getSpeeds() {
         return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }


public double getHeading(){
  return Math.IEEEremainder(gyro.getAngle(), 360);
}

public SwerveModuleState[] getModuleStates() {
  SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
  for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getState();
  }
  return states;
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

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

public void setModuleStates(SwerveModuleState[] desiredStates){
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  frontLeft.setDesiredState(desiredStates[0]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[3]);
}


}

