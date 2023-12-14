// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * 
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final int Driver = 0;
    public static final int Armor = 1;
    
    public class DriveTrainConstants
    {

        //Front Right
        public static final int FRA = 4;
        public static final int FRS = 3;

        //Front Left
        public static final int FLA = 1;
        public static final int FLS = 2;

        //Back Left
        public static final int BLA = 7;
        public static final int BLS = 8;

        //Back Right
        public static final int BRA = 5;
        public static final int BRS = 6;
   
    }

    public class ArmConstants
    {
        public static final int LowerArmR = 12;
        public static final int LowerArmL = 11;

        public static final int TopArm = 9;
    }

    public class EncoderConstants
    {

        public static final int LowerArmE = 0;
        public static final int TopArmE = 1;

        public static final int FRAE = 14;
        public static final int FLAE = 15;
        public static final int BLAE = 17;
        public static final int BRAE = 16;

    }

    public class RobotLengths
    {

        public static final double L = 21;
        public static final double W = 21;

    }
}
