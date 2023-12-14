/* 

Motor Ports:
FLA = 1
FLS = 2

FRA = 4
FRS = 3

BLA = 7
BLS = 8

BRA = 5
BRS = 6
 
Left Lower Arm = 11
Right Lower Arm = 12  MUST BE INVERTED

Top Arm = 13


Encoder Ports:
Lower Arm Duty Cycle = 0
Top Arm Duty Cycle = 1


Pneumatics:
Forward = 6
Reverse = 7
Hudson's Pneumatic = non functional

*/




































////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Code Graveyard //

///////////////Limelight/////////////////

//Lime Light Line Up

// public void limeLineUp(boolean toggleP, String color, double lowerPower, double topPower)
// {

//     table.getEntry("pipeline").setNumber(0);
//     //44in

//     if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//     {
//         targetHeight = 24.125;
//     }
//     else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//     {
//         targetHeight = 18.25;
//     }

//     x = horizontalOffset.getDouble(0.0);
//     y = verticalOffset.getDouble(0.0);
//     area = targetArea.getDouble(0.0);

//     angle = cameraAngle + y;
//     distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

//     if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//     {
//         varB = distance - 33;
//     }
//     else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//     {
//         varB = distance - 21;
//     }

//     varA = (distance * Math.tan(Math.toRadians(x)));
//     // System.out.println(varA);


//     if(driver.getRawButtonPressed(1) && lowerPower <= 0.05 && topPower <= 0.05)
//     {
//         if(color == "Yellow")
//         {
//         table.getEntry("pipeline").setNumber(0); //0 = Reflective Tape, 1 = AprilTag

//         Timer.delay(0.5);

        
//         robotTurn(180.0, true); //angular line up with the long white cylindrical object/////////////180
//         wheelStop();
//         // System.out.println("taco");

//         if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//         {
//             targetHeight = 24.125;
//         }
//         else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//         {
//             targetHeight = 18.25;
//         }

//         x = horizontalOffset.getDouble(0.0);
//         y = verticalOffset.getDouble(0.0);
//         area = targetArea.getDouble(0.0);

//         angle = cameraAngle + y;
//         distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

//         if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//         {
//             varB = distance - 35; //33
//         }
//         else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//         {
//             varB = distance - 21;
//         }

//         varA = (distance * Math.tan(Math.toRadians(x)));

//         if(x > 0)
//         {
//             wheelDistance(-90, -90, -90, -90, (varA + 2), true);
//             wheelStop();
//             // testVar = 1;
//         }
//         else if(x < 0)
//         {
//             wheelDistance(-90, -90, -90, -90, (varA + 1), true);
//             wheelStop();
//             // testVar = 2;
//         }

//         wheelDistance(0, 0, 0, 0, varB, true);
//         wheelStop();

//         // Diagonal Calculations
//         // limelightlineupwheelangle = -Math.toDegrees(Math.atan((distance * Math.tan(Math.toRadians(x))) / varB)); //29

//         // varA = (Math.sqrt(Math.pow(varB, 2) + Math.pow((distance * Math.tan(Math.toRadians(x))), 2))) - 3;
//         }
//         else
//         {

//         table.getEntry("pipeline").setNumber(1);

//         Timer.delay(0.5);

//         robotTurn(180, true); //angular line up with the Brown, short walled container
//         wheelStop();

//         if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//         {
//             targetHeight = 24.125;
//         }
//         else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//         {
//             targetHeight = 18.25;
//         }

//         Timer.delay(0.5);

//         x = horizontalOffset.getDouble(0.0);
//         y = verticalOffset.getDouble(0.0);
//         area = targetArea.getDouble(0.0);

//         angle = cameraAngle + y;
//         distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));
        
//         if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//         {
//             varB = distance - 35; //33
//         }
//         else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//         {
//             varB = distance - 21;
//         }

//         varA = (distance * Math.tan(Math.toRadians(x)));

//         if(x > 0)
//         {
//             wheelDistance(-90, -90, -90, -90, (varA + 2), true);
//             wheelStop();
//             // testVar = 1;

//         }
//         if(x < 0)
//         {
//             wheelDistance(-90, -90, -90, -90, (varA + 1), true);
//             wheelStop();
//             // testVar = 2;
//         }


//         wheelDistance(0, 0, 0, 0, varB, true);
//         wheelStop();
//         }
//     }
// }


// public void limeLineUpAuto(boolean toggleP, int targetType)
// {
//   table.getEntry("pipeline").setNumber(targetType);

//   Timer.delay(0.5);

//   if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//   {
//     targetHeight = 24.125;
//   }
//   else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//   {
//     targetHeight = 18.25;
//   }

//   x = horizontalOffset.getDouble(0.0);
//   y = verticalOffset.getDouble(0.0);
//   area = targetArea.getDouble(0.0);

//   angle = cameraAngle + y;
//   distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

//   varA = (distance * Math.tan(Math.toRadians(x)));


//   if(toggleP == true)
//   {
//     robotTurn(0.0, true); //angular line up with the long white cylindrical object/////////////180
//     wheelStop();

//     if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//     {
//       targetHeight = 24.125;
//     }
//     else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//     {
//       targetHeight = 18.25;
//     }

//     x = horizontalOffset.getDouble(0.0);
//     y = verticalOffset.getDouble(0.0);
//     area = targetArea.getDouble(0.0);

//     angle = cameraAngle + y;
//     distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

//     varA = (distance * Math.tan(Math.toRadians(x)));

//     if(x > 0)
//     {
//       wheelDistance(-90, -90, -90, -90, (varA + 4), true, null);
//       wheelStop();
//     }
//     if(x < 0)
//     {
//       wheelDistance(-90, -90, -90, -90, (varA + 1), true, null);
//       wheelStop();
//     }

//     angle = cameraAngle + y;
//     distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));
    
//     if(table.getEntry("pipeline").getDouble(5.0) == 0.0)
//     {
//       varB = distance - 31;
//     }
//     else if(table.getEntry("pipeline").getDouble(5.0) == 1.0)
//     {
//       varB = distance - 21;
//     }

//     wheelDistance(0, 0, 0, 0, varB, true, null);
//     wheelStop();
//   }  
// }

///////////LimeLight While Loop Work Around///////////

/*
   * test() will be constantly called in tele-op. Most of the time, testVar will equal 0, so test() will do nothing. When the
   * limelight line up button is pressed, testVar will be either set to 1 or 2, depending on wether the target is to the left 
   * or the right. *WARNING* testVar will be constantly set to 1 or 2 as long as the drivers hold the button. If the drivers 
   * let go of the button before the horizontal line up is complete, there shouldn't be a problem. Once testVar is set to 1 
   * or 2, the "onceOver" if statement will run ONCE, then the rest of the code (horizontal wheel distance line up) will be 
   * constantly run until the error requirements are met. At this point, testVar is set to 3. While testVar is equal to 3,
   * the same procedure will occur, accept the code will execute vertical wheel distance line up, then set testVar back to 0 
   * when done.
   */
  // public void test()
  // {
  //   if(testVar == 1)
  //   {
  //     if(onceOver == 1)
  //     {
  //       // rotations4degrees = (varA + 2) * 0.5722; //0.569875
  //       rotations4degrees = varA + 2;

  //       FLS.getEncoder().setPosition(0);
  //       FRS.getEncoder().setPosition(0);
  //       BLS.getEncoder().setPosition(0);
  //       BRS.getEncoder().setPosition(0);

  //       onceOver = 0;

  //       angelFL = -90;
  //       angelFR = -90;
  //       angelBL = -90;
  //       angelBR = -90;

  //       //get variables, store in new vars to avoid constant updates

        
  //     }
      
  //     // wheelAngles(angelFL, angelFR, angelBL, angelBR, true);


  //     averagedPositionDist = (FLS.getEncoder().getPosition() + FRS.getEncoder().getPosition() + BLS.getEncoder().getPosition() + BRS.getEncoder().getPosition()) / 4;

  //     // averagedCalcDist = pidVelocity.calculate(averagedPositionDist, rotations4degrees);
  //     // FLS.set(averagedCalcDist);
  //     // FRS.set(averagedCalcDist);
  //     // BLS.set(averagedCalcDist);
  //     // BRS.set(averagedCalcDist);

  //     smallestErrorDist = Math.abs(averagedPositionDist - rotations4degrees);

  //     // x = horizontalOffset.getDouble(0.0);
  //     // y = verticalOffset.getDouble(0.0);
  //     // area = targetArea.getDouble(0.0);

  //     // angle = cameraAngle + y;
  //     // distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

  //     wheelDistanceTEST(angelFL, angelFR, angelBR, angelBL, rotations4degrees, true);

  //     System.out.println("running1");

  //     if(testVar != 1)
  //     {
  //       testVar = 1; //trying to protect from wanted task changing mid line up
  //     }

  //     if(smallestErrorDist <= 0.05 || Math.abs(averagedCalcDist) <= 0.025 || driver.getRawButtonPressed(4))
  //     {
  //       testVar = 3;
  //       onceOver = 1;
        
  //       if(smallestErrorDist <= 0.05)
  //       {
  //         System.out.println(smallestErrorDist);
  //       }
  //       if(Math.abs(averagedCalcDist) <= 0.025)
  //       {
  //         System.out.println(averagedCalcDist);
  //       }
  //       
  //     }


  //   }
  //   else if(testVar == 2)
  //   {
  //     if(onceOver == 1)
  //     {
  //       // rotations4degrees = (varA + 1) * 0.5722; //0.569875
  //       rotations4degrees = varA + 1;

  //       FLS.getEncoder().setPosition(0);
  //       FRS.getEncoder().setPosition(0);
  //       BLS.getEncoder().setPosition(0);
  //       BRS.getEncoder().setPosition(0);

  //       onceOver = 0;

  //       angelFL = -90;
  //       angelFR = -90;
  //       angelBL = -90;
  //       angelBR = -90;

  //       //get variables, store in new vars to avoid constant updates


  //     }
      
  //     // wheelAngles(angelFL, angelFR, angelBL, angelBR, true);


  //     averagedPositionDist = (FLS.getEncoder().getPosition() + FRS.getEncoder().getPosition() + BLS.getEncoder().getPosition() + BRS.getEncoder().getPosition()) / 4;

  //     // averagedCalcDist = pidVelocity.calculate(averagedPositionDist, rotations4degrees);
  //     // FLS.set(averagedCalcDist);
  //     // FRS.set(averagedCalcDist);
  //     // BLS.set(averagedCalcDist);
  //     // BRS.set(averagedCalcDist);

  //     smallestErrorDist = Math.abs(averagedPositionDist - rotations4degrees);

  //     // x = horizontalOffset.getDouble(0.0);
  //     // y = verticalOffset.getDouble(0.0);
  //     // area = targetArea.getDouble(0.0);

  //     // angle = cameraAngle + y;
  //     // distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

  //     System.out.println("running2");

  //     wheelDistanceTEST(angelFL, angelFR, angelBR, angelBL, rotations4degrees, true);

  //     if(testVar != 2)
  //     {
  //       testVar = 2;
  //     }

  //     if(smallestErrorDist <= 0.05 || Math.abs(averagedCalcDist) <= 0.025 || driver.getRawButtonPressed(4))
  //     {
  //       testVar = 3;
  //       onceOver = 1;
  //       wheelStop();

  //       if(smallestErrorDist <= 0.05)
  //       {
  //         System.out.println(smallestErrorDist);
  //       }
  //       if(Math.abs(averagedCalcDist) <= 0.025)
  //       {
  //         System.out.println(averagedCalcDist);
  //       }
  //       
  //     }

  //   }
  //   else if(testVar == 3)
  //   {
  //     if(onceOver == 1)
  //     {
  //       // rotations4degrees = varB * 0.5722; //0.569875
  //       rotations4degrees = varB;

  //       FLS.getEncoder().setPosition(0);
  //       FRS.getEncoder().setPosition(0);
  //       BLS.getEncoder().setPosition(0);
  //       BRS.getEncoder().setPosition(0);

  //       onceOver = 0;

  //       angelFL = 0;
  //       angelFR = 0;
  //       angelBL = 0;
  //       angelBR = 0;

  //       //get variables, store in new vars to avoid constant updates

  //     }

  //     // wheelAngles(angelFL, angelFR, angelBL, angelBR, true);

  //     System.out.println("running3");
        

  //     averagedPositionDist = (FLS.getEncoder().getPosition() + FRS.getEncoder().getPosition() + BLS.getEncoder().getPosition() + BRS.getEncoder().getPosition()) / 4;

  //     // averagedCalcDist = pidVelocity.calculate(averagedPositionDist, rotations4degrees);
  //     // FLS.set(averagedCalcDist);
  //     // FRS.set(averagedCalcDist);
  //     // BLS.set(averagedCalcDist);
  //     // BRS.set(averagedCalcDist);

  //     smallestErrorDist = Math.abs(averagedPositionDist - rotations4degrees);

  //     // x = horizontalOffset.getDouble(0.0);
  //     // y = verticalOffset.getDouble(0.0);
  //     // area = targetArea.getDouble(0.0);

  //     // angle = cameraAngle + y;
  //     // distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));

  //     wheelDistanceTEST(angelFL, angelFR, angelBR, angelBL, rotations4degrees, true);

  //     if(testVar != 3)
  //     {
  //       testVar = 3;
  //     }

  //     if(smallestErrorDist <= 0.05 || Math.abs(averagedCalcDist) <= 0.025 || driver.getRawButtonPressed(4))
  //     {
  //       testVar = 0;
  //       onceOver = 1;
  //       wheelStop();
  //       if(smallestErrorDist <= 0.05)
  //       {
  //         System.out.println(smallestErrorDist);
  //       }
  //       if(Math.abs(averagedCalcDist) <= 0.025)
  //       {
  //         System.out.println(averagedCalcDist);
  //       }
  //       
        
  //     }

  //   }
  // }