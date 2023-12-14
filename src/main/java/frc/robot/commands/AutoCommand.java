// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrainClaw;

public class AutoCommand extends CommandBase {
  private DriveTrainClaw driveTrainClaw;
  private Arm arm;
  // private Shiny color;
  private int routine;
  private Timer timer;
  private double startTime;
  private final double runTime = 15;
  private boolean autoDone;

  private boolean hi;
  private boolean balanceToggle;

  private DriverStation driverStation;

  public AutoCommand(DriveTrainClaw subsystem, Arm sub2, int routine, boolean padBalance) {
    
    this.driveTrainClaw = subsystem;
    arm = sub2;
    // color = sub3;
    addRequirements(driveTrainClaw);
    timer = new Timer();
    this.routine = routine;

    hi = false;
    balanceToggle = padBalance;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    startTime = timer.get();
    autoDone = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() 
  {
    if(autoDone)
    {
      return;
    } 
    autoDone = true;

    // if(DriverStation.getAlliance() == DriverStation.Alliance.Blue)
    // {
    //   color.rainbow();
    // }
    // else if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
    // {
    //   color.fire();
    // }


    arm.ArmPosition(true, 1, timer); //middle cone position

    arm.ArmPosition(true, 5, timer); //high cone position
    arm.armHold(1);

    Timer.delay(.5);

    driveTrainClaw.clawRelease(); //drop cone

    Timer.delay(.3);

    arm.ArmPosition(true, 1, timer); //middle cone position

    arm.ArmPosition(true, 3, timer);
    arm.iReallyHopeThisWorks();

    driveTrainClaw.clawClose();


    if(routine == 1) //CP Blue
    {
      driveTrainClaw.wheelDistance(-179, -179, -179, -179, 150, true, timer); //150

      if(balanceToggle == true)
      {
        driveTrainClaw.wheelDistance(-90, -90, -90, -90, 85, true, timer); //60

        // color.setColor(225, 0, 0);
        driveTrainClaw.chargingPad(0, timer);
        // color.setColor(0, 225, 0);
      }
      

      driveTrainClaw.wheelStop();
      driveTrainClaw.driveTrainBrake();

    }
    if(routine == 2) //CP Red
    {
      driveTrainClaw.wheelDistance(-173, -173, -173, -173, 155, true, timer); //150

      if(balanceToggle == true)
      {
        driveTrainClaw.wheelDistance(-90, -90, -90, -90, 85, true, timer); //60

        // color.setColor(225, 0, 0);
        driveTrainClaw.chargingPad(0, timer);
        // color.setColor(0, 225, 0);
      }

      

      driveTrainClaw.wheelStop();
      driveTrainClaw.driveTrainBrake();
    }
    // else if(routine == 3)
    // {
    //   driveTrainClaw.wheelDistance(-175, -175, -175, -175, 220, true, timer);

    //   driveTrainClaw.clawRelease();

    //   driveTrainClaw.robotTurn(-8, true);
    //   driveTrainClaw.wheelStop();

    //   // driveTrainClaw.wheelDistance(0, 0, 0, 0, 6, true);

    //   // arm.ArmPosition(true, 5);

    //   // Timer.delay(0.2);

    //   // driveTrainClaw.clawClose();

    //   // Timer.delay(0.2);

    //   // arm.ArmPosition(true, 3);
    //   // arm.iReallyHopeThisWorks();

    //   // driveTrainClaw.robotTurn(180, true);

    //   // driveTrainClaw.wheelStop();
    //   // driveTrainClaw.driveTrainBrake();
    //   // System.out.println("***---...Done...---***");

    //   // Timer.delay(12);
    // }



    // FIRST TRUE AUTO, ARM AND ALL
    // arm.ArmPosition(true, 1);
    // arm.iReallyHopeThisWorks();
    // driveTrainClaw.wheelDistance(0, 0, 0, 0, 30, true);
    // driveTrainClaw.robotTurn(90, true);
    // arm.ArmPosition(true, 2);
    // driveTrainClaw.wheelDistance(0, 0, 0, 0, 30, true);
    // driveTrainClaw.robotTurn(90, true);
    // Timer.delay(15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainClaw.wheelStop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > startTime + runTime;
  }
}