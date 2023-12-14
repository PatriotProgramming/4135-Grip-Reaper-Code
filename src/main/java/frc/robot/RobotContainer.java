// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmCom;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainClaw;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  
  XboxController driver = new XboxController(Constants.Driver);
  XboxController armor = new XboxController(Constants.Armor);

  private final DriveTrainClaw driveTrainClaw = new DriveTrainClaw();
  private final Arm arm = new Arm();
  // private final Shiny shiny = new Shiny();
 

  private final ArmCom armCom = new ArmCom(arm, driver, armor);
  // private final LetThereBeLight lights = new LetThereBeLight(shiny);
  private final SwerveDrive swerveDrive = new SwerveDrive(driveTrainClaw, arm, driver);
  

  private final AutoCommand auto = new AutoCommand(driveTrainClaw, arm, 1, false);
//////////////1 = CP Blue (always use this one, will make it on bith sides)
//////////////2 = CP Red (no bump)
//////////////3 = Cube Pick Up

//////////////true = balance charging pad
//////////////false = only move out of community - no charging pad balance
//////////////works on both red and blue sides of the field


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    driveTrainClaw.setDefaultCommand(swerveDrive);
    arm.setDefaultCommand(armCom);
    // shiny.setDefaultCommand(lights);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    return auto;
  }
}
