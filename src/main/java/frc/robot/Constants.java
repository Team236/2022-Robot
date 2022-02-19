// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class ControllerConstants {
        public static final int USB_LEFT_STICK = 0;
        public static final int USB_RIGHT_STICK = 1;
        public static final int USB_CONTROLLER = 2;
        public static class Thrustmaster {
            public static final int TRIGGER = 1;
            public static final int BUTTON_MIDDLE = 2;
            public static final int BUTTON_LEFT = 3;
            public static final int BUTTON_RIGHT = 4;
            public static class AxesThrustmaster {
                public static final int X = 0;
                public static final int Y = 1;
                public static final int Z = 2;
                public static final int THROTTLE = 3;
            }       
        }
        public static class LogitechF310 {
            public static final int A = 2;
            public static final int B = 3;
            public static final int X = 1;
            public static final int Y = 4;
            public static final int LB = 5;
            public static final int RB = 6;
            public static final int BACK = 7;
            public static final int START = 8;
            public static final int LEFT_PRESS = 9;
            public static final int RIGHT_PRESS = 10;
            public class AxesController {
                public static final int LEFT_X = 0;
                public static final int LEFT_Y = 1;
                public static final int LT = 2;
                public static final int RT = 3;
                public static final int RIGHT_X = 4;
                public static final int RIGHT_Y = 5;
            }
        }
    }

    public static class MotorControllers {
        public static final int ID_LEFT_FRONT = 10;
        public static final int ID_LEFT_REAR = 11;
        public static final int ID_RIGHT_FRONT = 15;
        public static final int ID_RIGHT_REAR = 16;

        public static final int BOTTOM_SHOOTER = 3; //1;
        public static final int TOP_SHOOTER = 5; //7;

        public static final int INTAKE = 2; // this is a random number, need to get the actual ID
    }

    public static class Solenoids {

        public static final int INTAKE_SOL_FOR = 6;//CHECK THESE #s ON THE PCM
        public static final int INTAKE_SOL_REV = 7;

        public static final int SPOON_SOL_FOR = 2;
        public static final int SPOON_SOL_REV = 3;

    }
    public static class DriveConstants {
        public static final double LEFT_DEADZONE = 0.15;
        public static final double RIGHT_DEADZONE = 0.15;

        public static final double DIAMETER = 6; //4;
        public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
        public static final double GEAR_RATIO = 8.71 / 0.75;

        public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
        public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE; //multiply circumference by 0.75

        public static final boolean IS_DEADZONE = true;

        // PID
        public static double kP = 0.01;
        public static double turnkP = 0.016;
        public static double kI = 0;
        public static double kD = 0;
   
        public static double MIN_OUTPUT = -1;
        public static double MAX_OUTPUT = 1;
        public static final double MARGIN = 2;
        public static final double DISTANCE = 44;
        public static final double TURN_180 = 42;
        public static final double TURN_90 = 22;
	    
	    public static class limelight {
        // public static final double CATAPULTLEGNTH = ?;
        // public static final double CATAPULTANGLE = ?;
        // public static final double SPRINGCONSTANT = ?;   
        // public static final double CAMHEIGHT = ?;
        // public static final double CAMANGLE = ?;
        // public static final int HUBHEIGHT = 104;
        // public static final double TARMACDISTANCE = 84.75;
        // public static final int TARGETDIAMETER = 48;
        // public static final double TOTALANGLE = Math.tan((HUBHEIGHT-CAMHEIGHT) / TARMACDISTANCE);
        // public static final double CAMANGLECOMPLIMENT = TOTALANGLE - CAMANGLE;
        // public static final double TARGETAREA = Math.PI * TARGETDIAMETER;
        // public static final double TRUEDISTANCE = (HUBHEIGHT-CAMHEIGHT)/(Math.tan(CAMANGLE-CAMANGLECOMPLIMENT)); 
	    }
    }

    public static class ShooterConstants {
        public static final double kPBot = 0.0002;
        public static final double kIBot = 0.00000001;
        public static final double kDBot = 0.0565;
        public static final double kFFBot = 0.00021; //0.000215;//.00022

        public static final double kPTop = 0.0001;
        public static final double kITop = 0.00000001;
        public static final double kDTop = 0.0565;
        public static final double kFFTop = 0.00018; //0.000215;//.00022

        public static final double TOP_SPEED = 1125; // gear ratio is 18:32
        public static final double BOT_SPEED = 1125; // speed on wheels should be 2000

        public static final double HIGH_HUB_SMALL = 3234; //5750; //might need to swap these??
        public static final double HIGH_HUB_LARGE = 1530; //2720;

        public static final double LOW_HUB_SMALL = 1913; //3400;
        public static final double LOW_HUB_LARGE = 956; //1700;

        public static final double TARMAC_SMALL = 1237; // wheel speed = 2200;
        public static final double TARMAC_LARGE = 2261; // wheel speed = 4020;

        public static final double LAUNCH_PAD_SMALL = 6000;
        public static final double LAUNCH_PAD_LARGE = 3000;
    }

    public static class IntakeConstants {
        public static final int DIO_INTAKE_COUNTER = 0;

        public static final double FORWARD_SPEED = 0.3;
        public static final double REVERSE_SPEED = -0.3;
    }
    public static class ColorSensorConstants {
        public static final int DIST = 300;
    }
}
