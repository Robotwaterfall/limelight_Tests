// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveKickerCommand;
import frc.robot.commands.SwerveJoysickCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSub = new SwerveSubsystem();
  private final KickerSubsystem kickerSub = new KickerSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);

  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {

    swerveSub.setDefaultCommand(new SwerveJoysickCommand(
      swerveSub,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
    ));

    
  
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  
  private void configureBindings() {

    Command KickAlgae = new SequentialCommandGroup(

    new InstantCommand(() ->{
      kickerSub.setKickerSetpoint(150);

    }
    ));

    NamedCommands.registerCommand("KickAlgae", KickAlgae);
    
    new JoystickButton(driverJoystick, 2).whileTrue(new ZeroHeadingCommand(swerveSub));

    new JoystickButton(driverJoystick, 3).whileTrue(KickAlgae);

    SmartDashboard.putData("KickAlgaeStation3", new PathPlannerAuto("KickAlgaeStation3"));
  
  }

  public Command getAutonomousCommand(){

    return autoChooser.getSelected();
  }
}
