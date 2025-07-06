// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class ZeroHeadingCommand extends Command {

  private final SwerveSubsystem swerveSub;
  
  public ZeroHeadingCommand(SwerveSubsystem swerveSub) {
    this.swerveSub = swerveSub;

    addRequirements(swerveSub);
    
  }

  @Override
  public void execute() {

    swerveSub.zeroHeading();
  }


}
