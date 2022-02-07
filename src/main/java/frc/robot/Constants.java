// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class MainControllerConstants {

       // public static final int USB_LEFT_STICK = 0;
       // public static final int USB_RIGHT_STICK = 1;
        public static final int USB_CONTROLLER = 0;

        public static class Thrustmaster {
            public static final int joyTRIGGER = 1;
            public static final int joyBUTTON_MIDDLE = 2;
            public static final int joyBUTTON_LEFT = 3;
            public static final int joyBUTTON_RIGHT = 4;
    
            public static class AxesThrustmaster {
                public static final int joyX = 0;
                public static final int joyY = 1;
                public static final int joyZ = 2;
                public static final int joyTHROTTLE = 3;
            }
        }
        
        public static class LogitechF310 {
            public static final int contA = 1;
            public static final int contB = 2;
            public static final int contX = 3;
            public static final int contY = 4;
            public static final int contLB = 5;
            public static final int contRB = 6;
            public static final int contBACK = 7;
            public static final int contSTART = 8;
            public static final int contLEFT_PRESS = 9;
            public static final int contRIGHT_PRESS = 10;
            public class AxesController {
                public static final int contLEFT_X = 0;
                public static final int contLEFT_Y = 1;
                public static final int contLT = 2;
                public static final int contRT = 3;
                public static final int contRIGHT_X = 4;
                public static final int contRIGHT_Y = 5;
            }
        }
        

    }
    public static class DriveConstants {
        public static final int ID_LEFT_FRONT = 10;
        public static final int ID_LEFT_REAR = 11;
        public static final int ID_RIGHT_FRONT = 15;
        public static final int ID_RIGHT_REAR = 16;

        public static final double LEFT_DEADZONE = 0.15;
        public static final double RIGHT_DEADZONE = 0.15;

        public static final double DIAMETER = 6;
        public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
        public static final double GEAR_RATIO = 8.71;

        public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
        public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;

        public static final boolean IS_DEADZONE = true;
	    
	     public static class limelight {
       /*  public static final double CATAPULTLEGNTH = ?;
        public static final double CATAPULTANGLE = ?;
        public static final double SPRINGCONSTANT = ?;   
        public static final double CAMHEIGHT = ?;
        public static final double CAMANGLE = ?;
        public static final int HUBHEIGHT = 104;
        public static final double TARMACDISTANCE = 84.75;
        public static final int TARGETDIAMETER = 48;
        public static final double TOTALANGLE = Math.tan((HUBHEIGHT-CAMHEIGHT) / TARMACDISTANCE);
        public static final double CAMANGLECOMPLIMENT = TOTALANGLE - CAMANGLE;
        public static final double TARGETAREA = Math.PI * TARGETDIAMETER;
        public static final double TRUEDISTANCE = (HUBHEIGHT-CAMHEIGHT)/(Math.tan(CAMANGLE-CAMANGLECOMPLIMENT)); */
	}

        // PID
        public static double kP = 0.03;
        public static double kI = 0;
        public static double kD = 0;
        public static double kIz = 0;
        public static double kF = 0;

        public static double MIN_OUTPUT = -1;
        public static double MAX_OUTPUT = 1;
        public static final double MARGIN = 3;
        public static final double DISTANCE = 48;

    }

    public static class ColorSensorConstants {
        public static final I2C.Port i2cPort = I2C.Port.kOnboard;

        // public final Color BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
        // public final Color RED = ColorMatch.makeColor(0.343, 0.432, 0.223);
    }


    public static class TriggerConstants {
        public static final int TRIGGER_RETRACT = 1;
        public static final int TRIGGER_EXTEND = 0;
        
    }
}
