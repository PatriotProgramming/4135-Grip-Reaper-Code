// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.imageio.stream.MemoryCacheImageInputStream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private XboxController driver, armor;

  //Arm
  private DutyCycleEncoder lowerArmE, topArmE;

  private CANSparkMax topArm, lowerArmL, lowerArmR;

  private PIDController pidLowerArm, pidTopArm, pidLowerArmAuto;

  private double coneMiddleLowerArm, coneHighLowerArm;
  private double cubeMiddleLowerArm, cubeHighLowerArm;

  private double coneMiddleTopArm, coneHighTopArm;
  private double cubeMiddleTopArm, cubeHighTopArm;

  private double prePickBottomLowerArm, groundPickUpLowerArm;
  private double prePickBottomTopArm, groundPickUpTopArm;

  private double lowerArmPS, topArmPS;
  private double lowerArmHome, topArmHome;

  private double lowerArmWantedAngle, topArmWantedAngle;
  private double lowerArmAngle, topArmAngle;

  private double lowerArmFixPosition, topArmFixPosition;



  private double lowerArmEAngle, topArmEAngle;

  private boolean eSwitch;

  private double angleAngle, adjustedTopArmAngle, calculatedTopArmPower, calculatedLowerArmPower;

  public String hudsonSusday;
  
  public double myTest = 0;

  public int armDirection;

  private double driverRightX, driverRightY, driverLeftX, driverLeftY;
  private double armorRightX, armorRightY, armorLeftX, armorLeftY;

  private double lastTopArmWantedPos;

  private double p;

  private boolean pp;

  private double autoTopArmError, autoLowerArmError;

  private String manualMotorStatus;

  
  public Arm() 
  {
    driver = new XboxController(Constants.Driver);
    armor = new XboxController(Constants.Armor);

    //Arm Controllers
    

    topArm = new CANSparkMax(Constants.ArmConstants.TopArm, MotorType.kBrushless);
    topArm.setInverted(true);
    lowerArmL = new CANSparkMax(Constants.ArmConstants.LowerArmL, MotorType.kBrushless);
    lowerArmL.setInverted(true);
    lowerArmR = new CANSparkMax(Constants.ArmConstants.LowerArmR, MotorType.kBrushless);
    lowerArmR.setInverted(false);


    manualMotorStatus = "brake";
    //"brake" to set arm motors to brakeMode, "coast" to set arm motors to coastMode

    if(manualMotorStatus == "coast")
    {
      topArm.setIdleMode(IdleMode.kCoast);
      lowerArmL.setIdleMode(IdleMode.kCoast);
      lowerArmR.setIdleMode(IdleMode.kCoast);
    }
    else
    {
      topArm.setIdleMode(IdleMode.kBrake);
      lowerArmL.setIdleMode(IdleMode.kBrake);
      lowerArmR.setIdleMode(IdleMode.kBrake);
    }


    //Arm PID Controllers
    pidLowerArm = new PIDController(0.01, 0.002, 0); //0.01, 0.0017, 0
    pidLowerArm.disableContinuousInput();
    pidLowerArm.setIntegratorRange(-0.08, 0.15);
    pidTopArm = new PIDController(0.007, 0.0013, 0); //0.0057, 0.0013 , 0
    pidTopArm.disableContinuousInput();
    pidTopArm.setIntegratorRange(-0.08, 0.1);

    //Arm Encoders
    lowerArmE = new DutyCycleEncoder(Constants.EncoderConstants.LowerArmE);
    lowerArmE.setPositionOffset(50);
    topArmE = new DutyCycleEncoder(Constants.EncoderConstants.TopArmE);
    topArmE.setPositionOffset(50);

    //Arm Positions  //commented positions are for last claw
    coneMiddleLowerArm = 115.64; //111.5 
    coneMiddleTopArm = 94.22; // 113.0

    coneHighLowerArm = 133.36; // 135.97      //133.36
    coneHighTopArm = 151.93; //175.81         //148.93

    cubeMiddleLowerArm = 109.61; //105.69
    cubeMiddleTopArm = 87.04; //86.6


    cubeHighLowerArm = 131.26; //133.45
    cubeHighTopArm = 134.43; // 143.96

    lowerArmPS = 81.64; //75.64
    topArmPS = 71.5; //73.6

    lowerArmHome = 88; //82
    topArmHome = 18.71; //18.71

    
    groundPickUpLowerArm = 137.75; //127.83 last claw
    groundPickUpTopArm = 39.33; // 40.13

    prePickBottomLowerArm = 107.46; //107.46
    prePickBottomTopArm = 47.04; //47.04

    lowerArmFixPosition = 141.23; //141.23
    topArmFixPosition = 153.81; //153.81

    topArmWantedAngle = topArmHome;
    lowerArmWantedAngle = lowerArmHome;

    eSwitch = false;

    angleAngle = 0;
    adjustedTopArmAngle = 0;
    calculatedTopArmPower = 0;
    calculatedLowerArmPower = 0;

    armDirection = 0;

    //deadzoning
    driverRightX = 0;
    driverRightY = 0;
    driverLeftX = 0;
    driverLeftY= 0;

    armorRightX = 0;
    armorRightY = 0;
    armorLeftX = 0;
    armorLeftY= 0;

    lastTopArmWantedPos = 0;

    pp = false;

    autoTopArmError = 0;
    autoLowerArmError = 0;
  }

  public void joystickDeadzoning()
  {
    if(Math.abs(driver.getRightX()) < 0.05) //if joystick isn't being pressed far enough
    {
      driverRightX = 0.0; //set joystick to 0
    }
    else //if joystick is being pressed far enough
    {
      driverRightX = driver.getRightX(); //set joystick to read value
      //driverRightX = Math.pow(driver.getRightX(), 3);
    }

    if(Math.abs(driver.getRightY()) < 0.05)
    {
      driverRightY = 0.0;
    }
    else
    {
      driverRightY = driver.getRightY();
      //driverRightY = Math.pow(driver.getRightY(), 3);
    }

    if(Math.abs(driver.getLeftX()) < 0.05)
    {
      driverLeftX = 0.0;
    }
    else
    {
      driverLeftX = driver.getLeftX();
      //driverLeftX = Math.pow(driver.getLeftX(), 3);
    }

    if(Math.abs(driver.getLeftY()) < 0.05)
    {
      driverLeftY = 0.0;
    }
    else
    {
      driverLeftY = driver.getLeftY();
      //driverLeftY = Math.pow(driver.getLeftY(), 3);
    }


    if(Math.abs(armor.getRightX()) < 0.05)
    {
      armorRightX = 0.0;
    }
    else
    {
      armorRightX = armor.getRightX();
      //armorRightX = Math.pow(armor.getRightX(), 3);
    }

    if(Math.abs(armor.getRightY()) < 0.05)
    {
      armorRightY = 0.0;
    }
    else
    {
      armorRightY = armor.getRightY();
      //armorRightY = Math.pow(armor.getRightY(), 3);
    }

    if(Math.abs(armor.getLeftX()) < 0.05)
    {
      armorLeftX = 0.0;
    }
    else
    {
      armorLeftX = armor.getLeftX();
      //armorLeftX = Math.pow(armor.getLeftX(), 3);
    }

    if(Math.abs(armor.getLeftY()) < 0.05)
    {
      armorLeftY = 0.0;
    }
    else
    {
      armorLeftY = armor.getLeftY();
      //armorLeftY = Math.pow(armor.getLeftY(), 3);
    }
  }

  public void emergencySwitch()
  {
    if(armor.getRawButton(10))
    {
      if(eSwitch == false)
      {
        eSwitch = true;
      }
      else if(eSwitch == true)
      {
        eSwitch = false;
      }
    }
  }

  public void armRun() //moves arm to current wanted position
  {
    
    topArmAngle = topArmE.getAbsolutePosition() * 360;
    lowerArmAngle = (lowerArmE.getAbsolutePosition()) * 360; //reads current arm positions from encoders

    if(topArmAngle > 13 && topArmAngle < 174)
    {
      adjustedTopArmAngle = topArmAngle; //fix for broken encoder
    }

    if(armor.getRightTriggerAxis() > 0.5) //safety - only run when right trigger is held
    {
      // System.out.println(lowerArmAngle);
      if(lastTopArmWantedPos != topArmWantedAngle) //calculate direction of top arm movement
      {
        if(adjustedTopArmAngle > topArmWantedAngle)
        {
          armDirection = -1;
        }
        else if(adjustedTopArmAngle <= topArmWantedAngle)
        {
          armDirection = 1;
        }
  
        lastTopArmWantedPos = topArmWantedAngle;
      }

      if(armDirection == 1)
      {
        pidTopArm.setP(0.0135); //0.0135
        pidTopArm.setI(0.016); //0.007
        pidTopArm.setD(0.0001); //0.0001

        // SmartDashboard.putString("Top Arm PID Directiom", "Up");
      }
      else if(armDirection == -1)
      {
        pidTopArm.setP(0.01); //.015
        pidTopArm.setI(0.00013); //.00013
        pidTopArm.setD(0.0001); //0.0001

        // SmartDashboard.putString("Top Arm PID Directiom", "Down");
      }

      calculatedTopArmPower = pidTopArm.calculate(adjustedTopArmAngle, topArmWantedAngle) * 0.7; //PID Calculations for Top Arm
      calculatedLowerArmPower = pidLowerArm.calculate(lowerArmAngle, lowerArmWantedAngle) * 1.0; //PID Calculations for Lower Arm
      
      if(armDirection == 1)
      {
        if(calculatedTopArmPower >= 0.8)
        {
          calculatedTopArmPower = 0.8;
        }
      }
      else if(armDirection == -1)
      {
        if(calculatedTopArmPower >= 0.35)
        {
          calculatedTopArmPower = 0.35;
        }
      }

      topArm.set(-calculatedTopArmPower * 1);
      lowerArmL.set(calculatedLowerArmPower * 1);
      lowerArmR.set(calculatedLowerArmPower * 1);

    }
    else
    {
      topArm.stopMotor();
      lowerArmL.stopMotor();
      lowerArmR.stopMotor();
    }
  }

  public double lowerArmPower()
  {
    return calculatedLowerArmPower;
  }

  public double topArmPower()
  {
    return calculatedTopArmPower;
  }



  //Arm Positions
  public void armControl() //uses buttons to change position arm is moved to
  {
    if(armor.getRawButtonPressed(3)) //Middle Cone
    {
      topArmWantedAngle = coneMiddleTopArm;
      lowerArmWantedAngle = coneMiddleLowerArm;
      
    }
    else if(armor.getRawButtonPressed(4)) //Upper Cone
    {
      topArmWantedAngle = coneHighTopArm;
      lowerArmWantedAngle = coneHighLowerArm;
    }
    else if(armor.getRawButtonPressed(1)) //Floor
    {
      topArmWantedAngle = groundPickUpTopArm;
      lowerArmWantedAngle = groundPickUpLowerArm;
    }
    else if(armor.getRawButtonPressed(2)) //Substation
    {
      topArmWantedAngle = topArmPS;
      lowerArmWantedAngle = lowerArmPS;
    }
    else if(armor.getRawButtonPressed(7))
    {
      topArmWantedAngle = topArmHome;
      lowerArmWantedAngle = lowerArmHome;
    }
    else if(armor.getRawButton(5))
    {
      topArmWantedAngle = topArmFixPosition;
      lowerArmWantedAngle = lowerArmFixPosition;
    }
    else if(armor.getRawButtonPressed(9)) //Middle Cube CHANGE BUTTONS
    { 
      topArmWantedAngle = cubeMiddleTopArm;
      lowerArmWantedAngle = cubeMiddleLowerArm;
    }
    else if(armor.getRawButtonPressed(10)) //Upper Cube CHANGE BUTTONS
    {
      topArmWantedAngle = cubeHighTopArm;
      lowerArmWantedAngle = cubeHighLowerArm;
    }
  }


  //Arm Auto Positions
  public void ArmPosition(boolean ohio, int amazing, Timer tick) //Top Cone
  {

    //set wanted position based on input
    if(amazing == 1)
    {
      topArmWantedAngle = coneMiddleTopArm;
      lowerArmWantedAngle = coneMiddleLowerArm;
    }
    else if(amazing == 2)
    {
      topArmWantedAngle = cubeMiddleTopArm;
      lowerArmWantedAngle = cubeMiddleLowerArm;
    }
    else if(amazing == 3)
    {
      topArmWantedAngle = topArmHome;
      lowerArmWantedAngle = lowerArmHome;
    }
    else if(amazing == 4)
    {
      topArmWantedAngle = prePickBottomTopArm;
      lowerArmWantedAngle = prePickBottomLowerArm;
    }
    else if(amazing == 5)
    {
      topArmWantedAngle = coneHighTopArm;
      lowerArmWantedAngle = coneHighLowerArm;
    }
    else if(amazing == 6)
    {
      topArmWantedAngle = topArmFixPosition;
      lowerArmWantedAngle = lowerArmFixPosition;
    }
    else
    {
      ohio = false; //if no correct input, dont run arm
    }

    while(ohio == true) //run arm until deemed done
    {

      topArmAngle = topArmE.getAbsolutePosition() * 360;
      lowerArmAngle = (lowerArmE.getAbsolutePosition()) * 360; //read current angle off of arm encoders

      SmartDashboard.putNumber("Lower Arm Angle", lowerArmAngle);
      SmartDashboard.putNumber("Top Arm Angle", topArmAngle); //display arm encoders on SmartDashboard

      if(topArmAngle > 13 && topArmAngle < 174) //fix for broken arm encoder. encoder is no longer broken, but don't fix whats not broken
      {
        adjustedTopArmAngle = topArmAngle; //if read value is unreasonable (outside of possible arm movement), ignore value and use last read 
      }

      if(lastTopArmWantedPos != topArmWantedAngle) //when wanted top arm position changes
      {
        if(adjustedTopArmAngle > topArmWantedAngle) //if position is lower than last
        {
          armDirection = -1; //set direction as negative
        }
        else if(adjustedTopArmAngle <= topArmWantedAngle) //if position is higher than last
        {
          armDirection = 1; //set direction as positive
        }

        lastTopArmWantedPos = topArmWantedAngle; //refreshes last read wanted position
      }

      if(armDirection == 1) //if direction is positive, use pid tuned for fighting gravity
      {
        pidTopArm.setP(0.0135); //.0135
        pidTopArm.setI(0.01); //.007
        pidTopArm.setD(0.0001); //0.0001

        // SmartDashboard.putString("Top Arm PID Directiom", "Up");
      }
      else if(armDirection == -1) //if direction is negative, use pid tuned for working with gravity
      {
        pidTopArm.setP(0.01); //.015
        pidTopArm.setI(0.00013); //.00013
        pidTopArm.setD(0.0001); //0.001

        // SmartDashboard.putString("Top Arm PID Directiom", "Down");
      }

      calculatedTopArmPower = pidTopArm.calculate(adjustedTopArmAngle, topArmWantedAngle) * 0.7; //PID Calculations for Top Arm
      calculatedLowerArmPower = pidLowerArm.calculate(lowerArmAngle, lowerArmWantedAngle) * 1.0; //PID Calculations for Lower Arm
      
      if(armDirection == 1) //max arm speeds for fighting gravity
      {
        if(calculatedTopArmPower >= 0.8)
        {
          calculatedTopArmPower = 0.8;
        }
      }
      
      else if(armDirection == -1) //max arm speeds for working with gravity
      {
        if(calculatedTopArmPower >= 0.35)
        {
          calculatedTopArmPower = 0.35;
        }
      }

      topArm.set(-calculatedTopArmPower); 
      lowerArmL.set(calculatedLowerArmPower); //set arm motors to calculated speeds
      lowerArmR.set(calculatedLowerArmPower);

      autoTopArmError = Math.abs(adjustedTopArmAngle - topArmWantedAngle); //calculate error to wanted arm position
      autoLowerArmError = Math.abs(lowerArmAngle - lowerArmWantedAngle);

      if(autoTopArmError <= 4 && autoLowerArmError <= 4) //if both arm errors are small enough
      {
        ohio = false; //end arm loop
        topArm.set(0.05);
        lowerArmL.set(0.05); //feed forward to keep arm at current position
        lowerArmR.set(0.05);
      }

      if(tick.get() >= 14.98) //end loop early if auto ends to stop leak-over into tele-op
      {
        ohio = false;
        topArm.set(0.05);
        lowerArmL.set(0.05); //feed forward to keep arm at current position
        lowerArmR.set(0.05);
      }

      Timer.delay(0.02); //code runs too fast for pid tuned in tele-op. Delay deliberatly slows code to match tuned pid
      
    }
  }

  public void b() //possible arm adjustments for drivers
  {
    if(armor.getPOV() == 0) //up on d-pad
    {
      topArmWantedAngle += 0.05; //add to wanted arm position
    }
    else if(armor.getPOV() == 180) //down on d-pad
    {
      topArmWantedAngle -= 0.05; //subtract from wanted arm position
    }
  }


  public void iReallyHopeThisWorks() //apply power to keep arm at current position during auto
  {
    topArm.set((-calculatedTopArmPower + .02)); 
    lowerArmL.set((calculatedLowerArmPower + .01));
    lowerArmR.set((calculatedLowerArmPower + .01));
    // System.out.println(calculatedTopArmPower);
    // System.out.println(calculatedLowerArmPower);
    // System.out.println("-------------");
  }

  public void armHold(int position) //unfinished attempt to apply power to keep arm at current position during auto
  {
    if(position == 1) //Cone Middle
    {
      lowerArmL.set(-0.0116);
      lowerArmR.set(-0.0116);
      topArm.set(-0.07);
    }
    else if(position == 2) //Cube Middle
    {

    } //this code is unused, but also doesn't break anything so we're leaving it be
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Arm Encoders
    SmartDashboard.putNumber("Lower Arm Angle", lowerArmAngle);
    SmartDashboard.putNumber("Top Arm Angle", topArmAngle);

    SmartDashboard.putNumber("top arm power", calculatedTopArmPower);
    SmartDashboard.putNumber("lower arm power", calculatedLowerArmPower);
    // //pidTopArm.setD(extraLong);

    SmartDashboard.putNumber("Wanted Top Arm", topArmWantedAngle);
    SmartDashboard.putNumber("Wanted Lower Arm", lowerArmWantedAngle);

    // SmartDashboard.putNumber("MyTest", myTest);

    // SmartDashboard.putNumber("Corrected Top Arm Angle", adjustedTopArmAngle);

  }
}
