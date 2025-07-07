// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kickerConstants;

public class KickerSubsystem extends SubsystemBase {
  
  private final SparkMax kickerMotor = new SparkMax(kickerConstants.kKickerMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

  private final PIDController kickerController = new PIDController(kickerConstants.kKickerControllerKp, 
  kickerConstants.kKickerControllerKi, kickerConstants.kKickerControllerKd);

  private double kickerSetpoint;

  private boolean isHoldPostion;

  public KickerSubsystem() {

    kickerMotor.configure(kickerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    kickerController.enableContinuousInput(0, 360);

    kickerMotorConfig.absoluteEncoder.positionConversionFactor(360);
  }

  public SparkMax getKickerMotor(){
    return kickerMotor;
  }

  public PIDController getKickerController(){
    return kickerController;
  }

  public void setKickerSetpoint(double Setpoint_degress){
    kickerSetpoint = Setpoint_degress;
  }

  public double getKickerSetpoint(){
    return kickerSetpoint;
  }

  public boolean getIsHoldPostion(){
    return isHoldPostion;
  }

  public void setIsHoldPosition(boolean state){
    isHoldPostion = state;
  }


}
