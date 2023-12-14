// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainClaw extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private XboxController driver, armor;

  private AHRS navx;
  
  //Swerve Magic
  private double V1, V2, V3, V4, Vm;   //V final speed, V1 speed wheel 1, Vm = max velocity
  private double L, W, r, w, T; //Length and Width of robot, unit of measurement doesn't matter, R is hypotenuse of tan(L/W) //w rotation speed radians/s
  private double A, B, C, D; //4 equations needed
  private double FWD, STR, RCW, forward, straferight; //FWD forward, STR strafe right, RCW Rotate clockwise
  private double W1a, W2a, W3a, W4a; //Needed Wheel 1 angle, ect
  private double speed, rotations4degrees, dna, dnr; //Hudson is small
  private double temp, currentAngle;
  private double currentAngleFL, currentAngleFR, currentAngleBL, currentAngleBR;
  private double frontRightAngleEncoderDeg, frontLeftAngleEncoderDeg, backRightAngleEncoderDeg, backLeftAngleEncoderDeg;
  private double tempW1a, finalAngle1;
  private double tempW2a, finalAngle2;
  private double tempW3a, finalAngle3;
  private double tempW4a, finalAngle4;

  private CANSparkMax FLS, FRS, BLS, BRS, FLA, FRA, BLA, BRA; //each motor, Front Left Speed motor, Front Left Angle motor

  private CANCoder FLAE, FRAE, BLAE, BRAE; //encodors for each motor

  private PIDController pid, pidVelocity, pidBL;

  private boolean balanceCheck, balanceDone, onPad;

  //Claw
  private Compressor compressor;
  private DoubleSolenoid clawPinch;

  // LimeLight
  private NetworkTable table; //info from limelight

  private NetworkTableEntry horizontalOffset, verticalOffset, targetArea, validTarget;

  private double distance, cameraAngle = 26.5, cameraHeight = 9.75 , targetHeight = 24.125, x, y, area; //values for constant values

  private double directDist, horDist;

  public double childToucher;

  private double errorBL, errorBR, errorFR, errorFL;

  private double errorBLPP, errorBRPP, errorFRPP, errorFLPP;

  private double driverRightX, driverRightY, driverLeftX, driverLeftY;
  private double armorRightX, armorRightY, armorLeftX, armorLeftY;

  private double FLAAA, FRAAA, BLAAA, BRAAA;

  private double angle, limelightlineupwheelangle;

  private double smallestErrorDist;
  private double averagedPositionDist, averagedCalcDist;

  private double smallestErrorWA;
  private double averagedPositionWA, averagedCalcWA;

  private double varA, varB, varC, varD;

  private int testVar, onceOver;

  private double angelFR, angelFL, angelBR, angelBL, inchDist;

  private boolean F, M;

  private boolean driverMode;

  private double me;
  

  public DriveTrainClaw() 
  {
    // compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    driver = new XboxController(Constants.Driver);
    armor = new XboxController(Constants.Armor);

    //Front Right Module (Front Right Speed and Front Right Angle)
    FRS = new CANSparkMax(Constants.DriveTrainConstants.FRS, MotorType.kBrushless);
    FRS.setInverted(false);
    FRS.setSmartCurrentLimit(28); //28
    FRS.setIdleMode(IdleMode.kBrake);
    FRA = new CANSparkMax(Constants.DriveTrainConstants.FRA, MotorType.kBrushless);
    FRA.setSmartCurrentLimit(30);
    FRA.setIdleMode(IdleMode.kCoast);
    FRAE = new CANCoder(Constants.EncoderConstants.FRAE);

    //Front Left Module
    FLS = new CANSparkMax(Constants.DriveTrainConstants.FLS, MotorType.kBrushless);
    FLS.setInverted(true);
    FLS.setSmartCurrentLimit(28);
    FLS.setIdleMode(IdleMode.kBrake);
    FLA = new CANSparkMax(Constants.DriveTrainConstants.FLA, MotorType.kBrushless);
    FLA.setSmartCurrentLimit(30);
    FLA.setIdleMode(IdleMode.kCoast);  
    FLAE = new CANCoder(Constants.EncoderConstants.FLAE);

    //Back Left Module
    BLS = new CANSparkMax(Constants.DriveTrainConstants.BLS, MotorType.kBrushless);
    BLS.setInverted(true);
    BLS.setSmartCurrentLimit(28);
    BLS.setIdleMode(IdleMode.kBrake);
    BLA = new CANSparkMax(Constants.DriveTrainConstants.BLA, MotorType.kBrushless);
    BLA.setSmartCurrentLimit(30);
    BLA.setIdleMode(IdleMode.kCoast);
    BLAE = new CANCoder(Constants.EncoderConstants.BLAE);

    //Back Right Module
    BRS = new CANSparkMax(Constants.DriveTrainConstants.BRS, MotorType.kBrushless);
    BRS.setInverted(false);
    BRS.setSmartCurrentLimit(28);
    BRS.setIdleMode(IdleMode.kBrake);
    BRA = new CANSparkMax(Constants.DriveTrainConstants.BRA, MotorType.kBrushless);
    BRA.setSmartCurrentLimit(30);
    BRA.setIdleMode(IdleMode.kCoast);
    BRAE = new CANCoder(Constants.EncoderConstants.BRAE);

    //PID Controllers
    // pidBL = new PIDController(.005, 0, 0); //.005, 0, .00001
    // pidBL.enableContinuousInput(-180, 180);

    pid = new PIDController(.005, 0, 0.00001); //.0001, 0.00005, 0
    pid.enableContinuousInput(-180, 180);

    //pidVelocity = new PIDController(0.025, 0.002, 0.01); //0.025, 0.000019, 0.00001
    pidVelocity = new PIDController(0.025, 0.001, 0.008); //0.025 //0.001 //0.008
    pidVelocity.setIntegratorRange(-0.025, 0.025); //move I to maximum comfortable speed to tune
    

    //Robot Dimensions
    L = Constants.RobotLengths.L;
    W = Constants.RobotLengths.W;
    r = Math.sqrt((L*L) + (W*W));

    //navX
    navx = new AHRS();
    navx.zeroYaw();

    //Charging Pad
    balanceDone = false;
    balanceCheck = false;
    onPad = false;

    //Claw
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    clawPinch = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
    clawPinch.set(Value.kForward);
  
    //LimeLight
    table = NetworkTableInstance.getDefault().getTable("limelight-daza");
    horizontalOffset = table.getEntry("tx");
    verticalOffset = table.getEntry("ty");
    targetArea = table.getEntry("ta");
    validTarget = table.getEntry("tv");
    directDist = 0;

    //Auto Errors
    errorBL = 0;
    errorBR = 0;
    errorFL = 0;
    errorFR = 0;
    errorBLPP = 0;
    errorBRPP = 0;
    errorFLPP = 0;
    errorFRPP = 0;

    //deadzoning
    driverRightX = 0;
    driverRightY = 0;
    driverLeftX = 0;
    driverLeftY= 0;

    armorRightX = 0;
    armorRightY = 0;
    armorLeftX = 0;
    armorLeftY= 0;

    FLAAA = 0;
    FRAAA = 0;
    BLAAA = 0;
    BRAAA = 0;

    angle = 0;
    limelightlineupwheelangle = 0;

    averagedPositionDist = 0;
    averagedCalcDist = 0;

    averagedPositionWA = 0;
    averagedCalcWA = 0;

    varA = 0;
    varB = 0;
    varC = 0;
    varD = 0;

    CameraServer.startAutomaticCapture();

    childToucher = Math.atan(2);


    testVar = 0;
    onceOver = 1;

    angelFR = 0;
    angelFL = 0;
    angelBR = 0;
    angelBL = 0;
    inchDist = 0;
    
    F = true;
    M = true;

    driverMode = false; //hides most info in SmartDashboard from drivers
  }

  /////////////////////////////////////////////////////////////////////////////////
  
  public void joystickDeadzoning()
  {
    if(Math.abs(driver.getRightX()) < 0.05) //if joystick isn't being pressed far enough
    {
      driverRightX = 0.0; //set joystick to 0
    }
    else //if joystick is being pressed far enough
    {
      driverRightX = driver.getRightX(); //set joystick to read value
    }

    if(Math.abs(driver.getRightY()) < 0.05)
    {
      driverRightY = 0.0;
    }
    else
    {
      driverRightY = driver.getRightY();
    }

    if(Math.abs(driver.getLeftX()) < 0.05)
    {
      driverLeftX = 0.0;
    }
    else
    {
      driverLeftX = driver.getLeftX();
    }

    if(Math.abs(driver.getLeftY()) < 0.05)
    {
      driverLeftY = 0.0;
    }
    else
    {
      driverLeftY = driver.getLeftY();
    }
  }

  public void swerveMath()
  {

    // if(driver.getRawButton(5))
    // {
    //   table.getEntry("pipeline").setNumber(0);
    // }
    // if(driver.getRawButton(6))
    // {
    //   table.getEntry("pipeline").setNumber(1);
    // }


    currentAngleFR = FRAE.getAbsolutePosition();
    currentAngleFL = FLAE.getAbsolutePosition(); //get the current position of the swerve angle motors
    currentAngleBL = BLAE.getAbsolutePosition();
    currentAngleBR = BRAE.getAbsolutePosition();

    //Button Bindings
    forward = -driverLeftY;
    straferight = -driverLeftX;

    if(driverRightX > 0.05)
    {
      RCW = -driverRightX;
    }
    else if(driverRightX < -0.05)
    {
      RCW = -driverRightX;
    }
    else
    {
      RCW = 0;
    }
    
    double gyro_degrees = -navx.getYaw();
    double gyro_radians = gyro_degrees * (Math.PI / 180); 
    double temp = forward * Math.cos(gyro_radians) + straferight * Math.sin(gyro_radians); //makes drive field oriented
    STR = -forward * Math.sin(gyro_radians) + straferight * Math.cos(gyro_radians);
    FWD = temp;

    A = STR - RCW * (L/r);
    B = STR + RCW * (L/r);
    C = FWD - RCW * (W/r);
    D = FWD + RCW * (W/r);

    V1 = Math.sqrt(B*B + C*C); //calculates final velocity of motor
    W1a = Math.atan2(B,C) * 180/(Math.PI); //calculates angle of motor needed for resultant robot velocity and angle

    // V2x = B;
    // V2y = D;
    V2 = Math.sqrt(B*B + D*D);
    W2a = Math.atan2(B,D) * 180/(Math.PI);

    // V3x = A;
    // V3y = D;
    V3 = Math.sqrt(A*A + D*D);
    W3a = Math.atan2(A,D) * 180/(Math.PI);
    
    // V4x = A;
    // V4y = C;
    V4 = Math.sqrt(A*A + C*C);
    W4a = Math.atan2(A,C) * 180/(Math.PI);

    //normalizes velocities to be relative to eachother when Vm > 1
    Vm = V1;
    if(V2 > Vm)
    {
      Vm=V2;
    } 
    if(V3>Vm)
    {
      Vm=V3;
    } 
    if(V4>Vm)
    {
      Vm=V4;
    }
    if(Vm>1)
    {
      V1 /= Vm; //divide V1 by Vm
      V2 /= Vm; 
      V3 /= Vm; 
      V4 /= Vm;
    }

    frontRightAngleEncoderDeg = (((FRA.getEncoder().getPosition() % 1) + 1) % 1) * 360;
    frontLeftAngleEncoderDeg = (((FLA.getEncoder().getPosition() % 1) + 1) % 1) * 360; //keeps angle readings within 0-360 degree range
    backRightAngleEncoderDeg = (((BRA.getEncoder().getPosition() % 1) + 1) % 1) * 360;
    backLeftAngleEncoderDeg = (((BLA.getEncoder().getPosition() % 1) + 1) % 1) * 360;

    //WHEEL ANGLE FIXING

    // // FrontLeft
    // if(currentAngleFL - W1a > 180)
    // {
    //   tempW1a = (currentAngleFL - W1a) - 360; //if angle is farther than 180, loop back past -180
    // }
    // if(currentAngleFL - W1a <= -180)
    // {
    //   tempW1a = (currentAngleFL - W1a) + 360; //if angle is farther than -180, loop back past 180
    // }
    // if(Math.abs(tempW1a) >= 90) //if wanted angle is farther than 90 degrees, the coterminal angle must be closer
    // {
    //   if(W1a >= 0) 
    //   {
    //     finalAngle1 = W1a - 180; //find coterminal, that is the target angle
    //     V1 = -V1; //reverse speed because you are using the coterminal angle, which is opposite of the normal angle
    //   }
    //   else
    //   {
    //     finalAngle1 = W1a + 180;
    //     V1 = -V1;
    //   }
    // }
    // else
    // {
    //   finalAngle1 = W1a; //original angle is closer 
    // }

    // //Front Right
    // if(currentAngleFR - W2a > 180)
    // {
    //   tempW2a = (currentAngleFR - W2a) - 360;
    // }
    // if(currentAngleFR - W2a <= -180)
    // {
    //   tempW2a = (currentAngleFR - W2a) + 360;
    // }
    // if(Math.abs(tempW2a) >= 90)
    // {
    //   if(W2a >= 0)
    //   {
    //     finalAngle2 = W2a - 180;
    //     V2 = -V2;
    //   }
    //   else
    //   {
    //     finalAngle2 = W2a + 180;
    //     V2 = -V2;
    //   }
    // }
    // else
    // {
    //   finalAngle2 = W2a;
    // }

    // //BackRight
    // if(currentAngleBR - W3a > 180)
    // {
    //   tempW3a = (currentAngleBR - W3a) - 360;
    // }
    // if(currentAngleBR - W3a <= -180)
    // {
    //   tempW3a = (currentAngleBR - W3a) + 360;
    // }
    // if(Math.abs(tempW3a) >= 90)
    // {
    //   if(W3a >= 0)
    //   {
    //     finalAngle3 = W3a - 180;
    //     V3 = -V3;
    //   }
    //   else
    //   {
    //     finalAngle3 = W3a + 180;
    //     V3 = -V3;
    //   }
    // }
    // else
    // {
    //   finalAngle3 = W3a;
    // }

    // //Back Left
    // if(currentAngleBL - W4a > 180)
    // {
    //   tempW4a = (currentAngleBL - W4a) - 360;
    // }
    // if(currentAngleBL - W4a <= -180)
    // {
    //   tempW4a = (currentAngleBL - W4a) + 360;
    // }
    // if(Math.abs(tempW4a) >= 90)
    // {
    //   if(W4a >= 0)
    //   {
    //     finalAngle4 = W4a - 180;
    //     V4 = -V4;
    //   }
    //   else
    //   {
    //     finalAngle4 = W4a + 180;
    //     V4 = -V4;
    //   }
    // }
    // else
    // {
    //   finalAngle4 = W4a;
    // }


    if(driverLeftX == 0 && driverLeftY == 0 && driverRightX == 0)
    {
      W1a = currentAngleFL;
      W2a = currentAngleFR;
      W3a = currentAngleBR;
      W4a = currentAngleBL;
    }

    if(RCW == 0)
    {
      V1 = Math.pow(V1, 3);
      V2 = Math.pow(V2, 3);
      V3 = Math.pow(V3, 3);
      V4 = Math.pow(V4, 3);
    }
  }
  public void swerveDrive() //Make robot move with Swerve Drive -------------------------------
  {
    
    if(driver.getRightTriggerAxis() > 0.5) //safety trigger
    {
      FLA.set(pid.calculate(currentAngleFL, W1a)); //set angle motor power
      FLS.set(-V1 * .85); //set speed motor power

      FRA.set(pid.calculate(currentAngleFR, W2a));
      FRS.set(V2 * .85);

      BRA.set(pid.calculate(currentAngleBR, W3a));
      BRS.set(V3 * .85);

      BLA.set(pid.calculate(currentAngleBL, W4a));
      BLS.set(-V4 * .85);
    }
    else if(driver.getLeftTriggerAxis() > 0.5)
    {
      FLA.set(pid.calculate(currentAngleFL, W1a)); //set angle motor power
      FLS.set(-V1 * .25); //set speed motor power 

      FRA.set(pid.calculate(currentAngleFR, W2a));
      FRS.set(V2 * .25); // .15

      BRA.set(pid.calculate(currentAngleBR, W3a));
      BRS.set(V3 * .25);

      BLA.set(pid.calculate(currentAngleBL, W4a));
      BLS.set(-V4 * .25);
    }
    else
    {
      wheelStop(); //stop when safety is released
    }
  }

  public void turn180()
  {
    if(driver.getRawButton(2))
    {
      robotTurn(180, true);
    }
  }

  public void wheelAngles(double angleFL, double angleFR, double angleBL, double angleBR, boolean toggleW)
  {
    while(toggleW == true)
    {
      currentAngleFR = FRAE.getAbsolutePosition();
      currentAngleFL = FLAE.getAbsolutePosition(); //get current swerve angle motor positions
      currentAngleBR = BRAE.getAbsolutePosition();
      currentAngleBL = BLAE.getAbsolutePosition();

      FLAAA = pid.calculate(currentAngleFL, angleFL);
      FRAAA = pid.calculate(currentAngleFR, angleFR); //calculates power needed based on current posiition and wanted position
      BLAAA = pid.calculate(currentAngleBL, angleBL);
      BRAAA = pid.calculate(currentAngleBR, angleBR);

      FLA.set(FLAAA);
      FRA.set(FRAAA); //sets motors to calculated powers
      BLA.set(BLAAA);
      BRA.set(BRAAA);

      errorBL = Math.abs(currentAngleBL) - Math.abs(angleBL);
      errorBR = Math.abs(currentAngleBR) - Math.abs(angleBR); //calculates how far motors are currently off from target position
      errorFL = Math.abs(currentAngleFL) - Math.abs(angleFL);
      errorFR = Math.abs(currentAngleFR) - Math.abs(angleFR);

      smallestErrorWA = (errorBL + errorBR + errorFL + errorFR) / 4;
      // Timer.delay(0.02);
      
      if(smallestErrorWA <= 5)
      {
        toggleW = false; //end loop when motors are close enough to target positon
        // System.out.println("done");
      } 
    }
  }


  public void wheelDistance(double angleFL, double angleFR, double angleBL, double angleBR, double inches, boolean toggleL, Timer tick)
  { /////////////////////////////////////////////when moving forward in limelight line up, exit loop when press button
    // SmartDashboard.putNumber("FL Wanted Speed Power", pidVelocity.calculate(FLS.getEncoder().getPosition(), rotations4degrees));
    // SmartDashboard.putNumber("FR Wanted Speed Power", pidVelocity.calculate(FRS.getEncoder().getPosition(), rotations4degrees));
    // SmartDashboard.putNumber("BL Wanted Speed Power", pidVelocity.calculate(BLS.getEncoder().getPosition(), rotations4degrees));
    // SmartDashboard.putNumber("BR Wanted Speed Power", pidVelocity.calculate(BRS.getEncoder().getPosition(), rotations4degrees));

    rotations4degrees = inches * 0.5722; //0.569875

    FLS.getEncoder().setPosition(0);
    FRS.getEncoder().setPosition(0);
    BLS.getEncoder().setPosition(0);
    BRS.getEncoder().setPosition(0);

    while(toggleL == true)
    {
      wheelAngles(angleFL, angleFR, angleBL, angleBR, true);
      
      Timer.delay(0.005);

      averagedPositionDist = (FLS.getEncoder().getPosition() + FRS.getEncoder().getPosition() + BLS.getEncoder().getPosition() + BRS.getEncoder().getPosition()) / 4;

      averagedCalcDist = pidVelocity.calculate(averagedPositionDist, rotations4degrees);
      FLS.set(averagedCalcDist);
      FRS.set(averagedCalcDist);
      BLS.set(averagedCalcDist);
      BRS.set(averagedCalcDist);

      smallestErrorDist = Math.abs(averagedPositionDist - rotations4degrees);

      x = horizontalOffset.getDouble(0.0);
      y = verticalOffset.getDouble(0.0);
      area = targetArea.getDouble(0.0);

      angle = cameraAngle + y;
      distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

      if(smallestErrorDist <= 0.05 || Math.abs(averagedCalcDist) <= 0.025 || driver.getRawButtonPressed(4))
      {
        toggleL = false;
        wheelStop();
      }
      if(tick.get() >= 14.98)
      {
        toggleL = false;
        wheelStop();
      }
    }
  }


  public void wheelDistanceTEST(double angleFL, double angleFR, double angleBL, double angleBR, double inches, boolean toggleL)
  { //limelight

    // SmartDashboard.putNumber("FL Wanted Speed Power", pidVelocity.calculate(FLS.getEncoder().getPosition(), rotations4degrees));
    // SmartDashboard.putNumber("FR Wanted Speed Power", pidVelocity.calculate(FRS.getEncoder().getPosition(), rotations4degrees));
    // SmartDashboard.putNumber("BL Wanted Speed Power", pidVelocity.calculate(BLS.getEncoder().getPosition(), rotations4degrees));
    // SmartDashboard.putNumber("BR Wanted Speed Power", pidVelocity.calculate(BRS.getEncoder().getPosition(), rotations4degrees));

    rotations4degrees = inches * 0.553;

    FLS.getEncoder().setPosition(0);
    FRS.getEncoder().setPosition(0);
    BLS.getEncoder().setPosition(0);
    BRS.getEncoder().setPosition(0);

    

    wheelAngles(angleFL, angleFR, angleBL, angleBR, true);
    
    Timer.delay(0.005);

    

    FLS.set(pidVelocity.calculate(FLS.getEncoder().getPosition(), rotations4degrees));
    FRS.set(pidVelocity.calculate(FRS.getEncoder().getPosition(), rotations4degrees));
    BLS.set(pidVelocity.calculate(BLS.getEncoder().getPosition(), rotations4degrees));
    BRS.set(pidVelocity.calculate(BRS.getEncoder().getPosition(), rotations4degrees));

    // System.out.println(pidVelocity.calculate(FLS.getEncoder().getPosition(), rotations4degrees));

    errorBLPP = Math.abs(BLS.getEncoder().getPosition() - rotations4degrees);
    errorBRPP = Math.abs(BRS.getEncoder().getPosition() - rotations4degrees);
    errorFLPP = Math.abs(FLS.getEncoder().getPosition() - rotations4degrees);
    errorFRPP = Math.abs(FRS.getEncoder().getPosition() - rotations4degrees);

    // System.out.println(errorBLPP);
    // System.out.println(errorBRPP);
    // System.out.println(errorFLPP);
    // System.out.println(errorFRPP);
    
  }

  public void wheelPowerMove(double power)
  {
    FLS.set(power); 
    FRS.set(power);
    BLS.set(power);
    BRS.set(power);   
  }

  public void robotTurn(double degrees, boolean toggle) //positive rotational movemement. H_ud_son sm_all  Input = desired angle
  { 
    while(toggle == true)
    {
      // System.out.println("oiuytre");

      dna = navx.getYaw() - degrees;

      if(dna >= 180)
      {
        dna -= 360;
      }
      else if(dna < -180)
      {
        dna += 360;
      }

      dnr = dna * 0.2783;

      wheelDistanceTEST(135, 45, -135, -45, dnr, true);

      if(Math.abs(degrees - navx.getYaw()) <= 3)
      {
        toggle = false;
      }
    }
    
  }

  public void testinggggg()
  {
    if(driver.getRawButton(5))
    {
      FLA.set(0.1);
      FRA.set(0.1);
      BLA.set(0.1);
      BRA.set(0.1);
    }
    else
    {
      FLA.set(0);
      FRA.set(0);
      BLA.set(0);
      BRA.set(0);
    }
  }

  public void chargingPad(double angleTF, Timer tick)
  {
    // robotTurn(angleTF, true);

    // navx.zeroYaw();

    // wheelAngles(0);

    

    while(balanceDone == false)
    {
      // SmartDashboard.putNumber("roll", navx.getRoll());

      if(tick.get() >= 14.98)
      {
        return;
      }
      
      
      while(balanceCheck == false)
      {

        if(tick.get() >= 14.98)
        {
          return;
        }

        // SmartDashboard.putNumber("roll", navx.getRoll());

        while(onPad == false)
        {
          // SmartDashboard.putNumber("roll", navx.getRoll());
          if(tick.get() >= 14.98)
          {
            return;
          }

          wheelAngles(0, 0, 0, 0, true);
          wheelPowerMove(0.5); //0.35

          if(navx.getRoll() >= 9 || navx.getRoll() <= -9)
          { 
            onPad = true;
            wheelStop();

            wheelDistance(0, 0, 0, 0, 20.0, true, tick);
            Timer.delay(0.413);
          }
        } 

      


        wheelAngles(0, 0, 0, 0, true);
        if(navx.getRoll() >= 6)
        {
          wheelAngles(0, 0, 0, 0, true);
          wheelPowerMove(0.11 * (navx.getRoll() / Math.abs(navx.getRoll()))); // 0.65
        }
        else if(navx.getRoll() <= -6)
        {
          wheelAngles(0, 0, 0, 0, true);
          wheelPowerMove(0.11 * (navx.getRoll() / Math.abs(navx.getRoll()))); //0.85
        }

        /*
         * 
         * wheelAngles(0, 0, 0, 0, true);
          if(navx.getRoll() >= 6)
          {
            wheelAngles(0, 0, 0, 0, true);
            wheelPowerMove(0.2 * (navx.getRoll() / Math.abs(navx.getRoll()))); // 0.65
          }
          else if(navx.getRoll() <= -6)
          {
            wheelAngles(0, 0, 0, 0, true);
            wheelPowerMove(0.2 * (navx.getRoll() / Math.abs(navx.getRoll()))); //0.85
          }

          Timer.delay(0.2);
          wheelStop();
          Timer.delay(0.2);
         * 
         */

        if(navx.getRoll() <= 6 && navx.getRoll() >= -6) //3, -3
        {
          balanceCheck = true;
          wheelStop();
          // wheelAngles(135, 45, -135, -45, true);

        }

        if(tick.get() >= 14.98)
        {
          return;
        }
      }

      Timer.delay(0.5);

      if(navx.getRoll() > 6 || navx.getRoll() < -6) //3, -3
      {
        balanceCheck = false;
        // System.out.println("redo");
        // wheelAngles(0, 0, 0, 0, true);
      }
      else
      {
        balanceDone = true;
        // System.out.println("done");
        // wheelAngles(135, 45, -135, -45, true);
      }
    }
  }



  


  
  public void zeroYaw()
  {
    if(driver.getRawButton(7))
    {
      navx.zeroYaw();
    }
  }

  public void zeroYawAuto()
  {
      navx.zeroYaw();
  }




  //Auto Routine Zeros and Stops
  public void wheelZero()
  {
    FLS.getEncoder().setPosition(0);
    FRS.getEncoder().setPosition(0);
    BLS.getEncoder().setPosition(0);
    BRS.getEncoder().setPosition(0);
  }

  public void wheelStop()
  {
    FRS.stopMotor();
    FLS.stopMotor();
    BLS.stopMotor();
    BRS.stopMotor();
    FRA.stopMotor();
    FLA.stopMotor();
    BLA.stopMotor();
    BRA.stopMotor();
  }

  public void driveTrainCoast()
  {
    FRS.setIdleMode(IdleMode.kCoast);
    FLS.setIdleMode(IdleMode.kCoast);
    BRS.setIdleMode(IdleMode.kCoast);
    BLS.setIdleMode(IdleMode.kCoast);
  }

  public void driveTrainBrake()
  {
    FRS.setIdleMode(IdleMode.kBrake);
    FLS.setIdleMode(IdleMode.kBrake);
    BRS.setIdleMode(IdleMode.kBrake);
    BLS.setIdleMode(IdleMode.kBrake);
  }

  public void clawRelease()
  {
    clawPinch.set(Value.kReverse);
  }

  public void clawClose()
  {
    clawPinch.set(Value.kForward);
  }

  public void moveClaw() //Toggle claw open and closed ---------------------------------------
  {
    if(armor.getRawButtonPressed(8))
    {
      clawPinch.toggle();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(driverMode == false)
    {
      // SWERVE
      
      SmartDashboard.putNumber("Front Right Angle CANcoder", FRAE.getAbsolutePosition());
      SmartDashboard.putNumber("Front Left Angle CANcoder", FLAE.getAbsolutePosition()); //absolute external encoders of swerve angle motors
      SmartDashboard.putNumber("Back Left Angle CANcoder", BLAE.getAbsolutePosition());
      SmartDashboard.putNumber("Back Right Angle CANcoder", BRAE.getAbsolutePosition());

      // SmartDashboard.putNumber("Front Right Angle motor encoder", frontRightAngleEncoderDeg);
      // SmartDashboard.putNumber("Front Left Angle motor encoder", frontLeftAngleEncoderDeg); //relative internal encoders of swerve angle motors
      // SmartDashboard.putNumber("Back Left Angle motor encoder", backRightAngleEncoderDeg);
      // SmartDashboard.putNumber("Back Right Angle motor encoder", backLeftAngleEncoderDeg);

      // SmartDashboard.putNumber("Front Left Speed Encoder", FLS.getEncoder().getPosition());
      // SmartDashboard.putNumber("Front Right Speed Encoder", FRS.getEncoder().getPosition()); //relative internal encoders of swerve speed motors
      // SmartDashboard.putNumber("Back Left Speed Encoder", BLS.getEncoder().getPosition());
      // SmartDashboard.putNumber("Back Right Speed Encoder", BRS.getEncoder().getPosition());

      // SmartDashboard.putNumber("A", A);
      // SmartDashboard.putNumber("B", B); //Swerve calculations
      // SmartDashboard.putNumber("C", C);
      // SmartDashboard.putNumber("D", D);

      SmartDashboard.putNumber("Wanted 1 angle", W1a);
      SmartDashboard.putNumber("Wanted 2 angle", W2a); //target angle of swerve angle motors
      SmartDashboard.putNumber("Wanted 3 angle", W3a);
      SmartDashboard.putNumber("Wanted 4 angle", W4a);

      // SmartDashboard.putNumber("V1", V1);
      // SmartDashboard.putNumber("V2", V2); //target speed of swerve speed motors
      // SmartDashboard.putNumber("V3", V3);
      // SmartDashboard.putNumber("V4", V3);
      
      // swerve
      // SmartDashboard.putNumber("Front Left Angle Power", pid.calculate(currentAngleFR, W1a));
      // SmartDashboard.putNumber("Front Right Angle Power", pid.calculate(currentAngleFL, W2a)); //power directed towards swerve angle motors
      // SmartDashboard.putNumber("Back Left Angle Power", pid.calculate(currentAngleBL, W3a));
      // SmartDashboard.putNumber("Back Right Angle Power", pid.calculate(currentAngleBR, W4a));
      
      // SmartDashboard.putNumber("FWD", FWD);
      // SmartDashboard.putNumber("STR", STR); //inputs into swerve calculations
      // SmartDashboard.putNumber("RCW", RCW);

      SmartDashboard.putNumber("Child Toucher", childToucher); //self explanatory

      // SmartDashboard.putNumber("FL Wanted Angle Power", FLAAA);
      // SmartDashboard.putNumber("FR Wanted Angle Power", FRAAA); //power directed towards swerve angle motors but again
      // SmartDashboard.putNumber("BL Wanted Angle Power", BLAAA);
      // SmartDashboard.putNumber("BR Wanted Angle Power", BRAAA);

      // SmartDashboard.putNumber("Angle", varD); 
      // SmartDashboard.putNumber("Wheels Turn Angle", limelightlineupwheelangle); //limelight line up calculations
      // SmartDashboard.putNumber("varA", varA);
      // SmartDashboard.putNumber("varB", varB);
      // SmartDashboard.putNumber("Vert Offset", y);
      // SmartDashboard.putNumber("Angle", angle);
      // SmartDashboard.putNumber("Poop", limelightlineupwheelangle);

    //   SmartDashboard.putNumber("roll", navx.getRoll()); //navx roll for charging pad balance

    //   SmartDashboard.putNumber("Front Left Velocity", FLS.getEncoder().getVelocity());
    //   SmartDashboard.putNumber("Front Right Velocity", FRS.getEncoder().getVelocity());
    //   SmartDashboard.putNumber("Back Left Velocity", BLS.getEncoder().getVelocity());
    //   SmartDashboard.putNumber("Back Right Velocity", BRS.getEncoder().getVelocity());
    } 

    // SmartDashboard.putNumber("DistanceKP", distance);
    // SmartDashboard.putNumber("Hor Offset", x);
    // SmartDashboard.putNumber("NAVX Angle", navx.getYaw()); //info for drivers
  }
}