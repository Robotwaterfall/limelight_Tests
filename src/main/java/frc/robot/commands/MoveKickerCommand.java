// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kickerConstants;
import frc.robot.subsystems.KickerSubsystem;


public class MoveKickerCommand extends Command {

  private KickerSubsystem kickerSub;

  private SparkMax kickerMotor;

  private PIDController kickerController;
  
  public MoveKickerCommand(KickerSubsystem kickerSub, SparkMax kickerMotor, PIDController kickerController) {

    this.kickerSub = kickerSub;
    this.kickerMotor = kickerSub.getKickerMotor();
    this.kickerController = kickerSub.getKickerController();
    addRequirements(kickerSub);
    
  }

  
  @Override
  public void initialize() {

    kickerMotor.set(0);
    kickerMotor.stopMotor();
  }

  
  @Override
  public void execute() {

    double kickerCurrentPosition = kickerMotor.getAbsoluteEncoder().getPosition();

    double output = kickerController.calculate(kickerCurrentPosition, kickerSub.getKickerSetpoint());
    if(Math.abs(output) >= kickerConstants.kKickerMaxPower)
    {
      output = kickerConstants.kKickerMaxPower * Math.signum(output);
    }

    kickerMotor.set(output);
  }

  
  @Override
  public void end(boolean interrupted) {

    kickerMotor.set(0);
    kickerMotor.stopMotor();
  }

  
  @Override
  public boolean isFinished() {

    return false;

  }
}
