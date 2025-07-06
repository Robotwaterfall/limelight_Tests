// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoysickCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSub = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
  
  public RobotContainer() {

    swerveSub.setDefaultCommand(new SwerveJoysickCommand(
      swerveSub,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
    ));
  
    configureBindings();
  }

  
  private void configureBindings() {
    
    new JoystickButton(driverJoystick, 2).whileTrue(new ZeroHeadingCommand(swerveSub));
  
  }

  public Command getAutonomousCommand(){

    return null;
  }
}
