// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainClaw;

public class ArmCom extends CommandBase {

  private Arm arm;
  private XboxController driver, armor;
  
  /** Creates a new ArmCom. */
  public ArmCom(Arm subsystem, XboxController driver, XboxController armor) 
  {
    arm = subsystem;
    addRequirements(arm);
    this.armor = armor;
    this.driver = driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    arm.joystickDeadzoning();
    arm.armControl();
    arm.emergencySwitch();
    arm.armRun();
    // arm.test();
    // your mom 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
