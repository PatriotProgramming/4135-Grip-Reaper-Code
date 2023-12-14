// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.sql.Time;

// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdleConfiguration;
// import com.ctre.phoenix.led.ColorFlowAnimation;
// import com.ctre.phoenix.led.FireAnimation;
// import com.ctre.phoenix.led.LarsonAnimation;
// import com.ctre.phoenix.led.RainbowAnimation;
// import com.ctre.phoenix.led.RgbFadeAnimation;
// import com.ctre.phoenix.led.SingleFadeAnimation;
// import com.ctre.phoenix.led.StrobeAnimation;
// import com.ctre.phoenix.led.TwinkleAnimation;
// import com.ctre.phoenix.led.TwinkleOffAnimation;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
// import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Shiny extends SubsystemBase {
//   /** Creates a new Shiny. */

//   //LED Lights

//   private CANdle lights;
//   private CANdleConfiguration lightConfig;

//   private ColorFlowAnimation colorFlowAnim;
//   private RainbowAnimation rainbowAnim;
//   private FireAnimation fireAnim;
//   private StrobeAnimation strobeAnim;
//   private TwinkleAnimation twinkleAnim;
//   private TwinkleOffAnimation twinkleOffAnim;
//   private LarsonAnimation larsonAnim1, larsonAnim2;
//   private RgbFadeAnimation rgbFadeAnim;
//   private SingleFadeAnimation singleFadeAnim;
  

//   private XboxController driver;
//   private boolean toggle;

//   private DriverStation driverStation;

//   private AHRS navx;

//   private boolean var1, var2, var3;

//   private Timer timer;

  
//   public Shiny() 
//   {
//     lights = new CANdle(1, "rio");
//     lightConfig = new CANdleConfiguration();

//     lightConfig.stripType = LEDStripType.GRB;
//     lightConfig.vBatOutputMode = VBatOutputMode.Modulated;
//     lightConfig.brightnessScalar = 100;
    
//     lights.configAllSettings(lightConfig);

//     colorFlowAnim = new ColorFlowAnimation(100, 10, 100);
//     rainbowAnim = new RainbowAnimation(1, 0.5, 320);
//     fireAnim = new FireAnimation(1, 1, -1, 1, 1, false, 205);
//     strobeAnim = new StrobeAnimation(0, 0, 0, 200, 0.5, 320);
//     twinkleAnim = new TwinkleAnimation(10, 0, 100, 100, 0.5, 320, TwinklePercent.Percent100);
//     twinkleOffAnim = new TwinkleOffAnimation(10, 0, 100, 100, 0.5, 320, TwinkleOffPercent.Percent100);
//     larsonAnim1 = new LarsonAnimation(255, 0, 0, 0, .5, 320, LarsonAnimation.BounceMode.Front, 40);
//     larsonAnim2 = new LarsonAnimation(0, 0, 255, 0, .5, 320, LarsonAnimation.BounceMode.Front, 40);
//     rgbFadeAnim = new RgbFadeAnimation(1,.5,-1, 0);
//     singleFadeAnim = new SingleFadeAnimation(0, 0, 200, 0, 0.5, 320);

//     driver = new XboxController(0);
//     toggle = false;

//     navx = new AHRS();
    
//     var1 = false;
//     var2 = false;
//     var3 = false;

//     // timer.reset();
//     // timer.start();
//     lights.clearAnimation(0);
//   }


//   public void shine()
//   {
//     if(driver.getRawButtonPressed(1))
//     {
//       var1 = false;
//       if(toggle == false)
//       {
//         lights.clearAnimation(0);
//         lights.setLEDs(255, 234, 3);
//         toggle = true;
//       }
//       else if(toggle == true)
//       {
//         lights.clearAnimation(0);
//         lights.setLEDs(120, 3, 255);
//         toggle = false;
//       }
//     }
//     else if(driver.getRawButtonPressed(8))
//     {
//       var1 = false;
//       if(DriverStation.getAlliance() == DriverStation.Alliance.Blue)
//       {
//         lights.clearAnimation(0);
//         rainbow();
//       }
//       else if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
//       {
//         lights.clearAnimation(0);
//         fire();
//       }
//     }
//     else if(var1 == false && Math.abs(navx.getRoll()) >= 9) //if drive on charging pad
//     {
//       lights.clearAnimation(0);
//       lights.setLEDs(225, 0, 0); //be red
//       var1 = true; //tell code we are balancing
//     }
//     else if(var1 == true && var2 == false && Math.abs(navx.getRoll()) <= 3) //if balancing and flat
//     {
//       lights.clearAnimation(0);
//       lights.setLEDs(0, 225, 0); //be green
//       var2 = true; //tell code we are flat
      
//     }
//     else if(var1 == true && var2 == true && Math.abs(navx.getRoll()) >= 3) //if balancing and angled
//     {
//       lights.clearAnimation(0);
//       lights.setLEDs(225, 0, 0); //be red
//       var2 = false; //tell code we are angled
//     }
//   }



//   public void larson1()
//   {
//     // lights.clearAnimation(0);
//     lights.animate(larsonAnim1);
//   }
//   public void larson2()
//   {
//     // lights.clearAnimation(0);
//     lights.animate(larsonAnim2);
//   }




  
//   public void colorFlow()
//   {
//     lights.clearAnimation(0);
//     lights.animate(colorFlowAnim);
//   }
//   public void rainbow()
//   {
//     lights.clearAnimation(0);
//     lights.animate(rainbowAnim);
//   }
//   public void fire()
//   {
//     lights.clearAnimation(0);
//     lights.animate(fireAnim);
//   }
//   public void strobe()
//   {
//     lights.clearAnimation(0);
//     lights.animate(strobeAnim);
//   }
//   public void twinkle()
//   {
//     lights.clearAnimation(0);
//     lights.animate(twinkleAnim);
//   }
//   public void twinkleOff()
//   {
//     lights.clearAnimation(0);
//     lights.animate(twinkleOffAnim);
//   }
//   public void rgbFade()
//   {
//     lights.clearAnimation(0);
//     lights.animate(rgbFadeAnim);
//   }
//   public void singleFade()
//   {
//     lights.clearAnimation(0);
//     lights.animate(singleFadeAnim);
//   }
//   public void setColor(int red, int green, int blue)
//   {
//     lights.clearAnimation(0);
//     lights.setLEDs(red, green, blue);
//   }
//   public void off()
//   {
//     lights.clearAnimation(0);
//     lights.setLEDs(0, 0, 0);
//   }

  




//   // public void balanceLight()
//   // {
//   //   if(timer.get() >= 1) //only run in endgame
//   //   {
//   //     if(var1 == false && Math.abs(navx.getRoll()) >= 3) //if drive on charging pad //9
//   //     {
//   //       lights.clearAnimation(0);
//   //       lights.setLEDs(225, 0, 0); //be red
//   //       var1 = true; //tell code we are balancing
//   //     }
//   //     else if(var1 == true && var2 == false && Math.abs(navx.getRoll()) <= 3) //if balancing and flat
//   //     {
//   //       lights.clearAnimation(0);
//   //       lights.setLEDs(0, 225, 0); //be green
//   //       var2 = true; //tell code we are flat
//   //     }
//   //     else if(var1 == true && var2 == true && Math.abs(navx.getRoll()) >= 3) //if balancing and angled
//   //     {
//   //       lights.clearAnimation(0);
//   //       lights.setLEDs(225, 0, 0); //be red
//   //       var2 = false; //tell code we are angled
//   //     }
//   //   }
//   // }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
