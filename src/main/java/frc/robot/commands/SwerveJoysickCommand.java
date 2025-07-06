// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveJoysickCommand extends Command {
  private final SwerveSubsystem swerveSub;
  private final Supplier <Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier <Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
 
  public SwerveJoysickCommand(SwerveSubsystem swerveSub, Supplier <Double> xSpdFunction, Supplier <Double> ySpdFunction,
  Supplier <Double> turningSpdFunction, Supplier <Boolean> fieldOrientedFunction) {
    this.swerveSub = swerveSub;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(swerveSub);
    
  }


  @Override
  public void initialize() {}

  
  @Override
  public void execute() {

    //1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    //2. Apply Deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    //3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;

    //4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunction.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 
                  turningSpeed, swerveSub.getRotation2d());
    }else{
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    //5. Convert the chassis speeds to indivual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //6. Output the module states to each wheel
    swerveSub.setModuleStates(moduleStates);


  }

  
  @Override
  public void end(boolean interrupted) {

    swerveSub.stopModules();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
