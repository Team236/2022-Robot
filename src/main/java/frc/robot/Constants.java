// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
            public static final int LEFT_BASE_1 = 11;
            public static final int LEFT_BASE_2 = 16;
            public static final int LEFT_BASE_3 = 13;
            public static final int LEFT_BASE_4 = 14;
            public static final int RIGHT_BASE_5 = 7;
            public static final int RIGHT_BASE_6 = 8;
            public static final int RIGHT_BASE_7 = 5;
            public static final int RIGHT_BASE_8 = 10;

            public static class AxesThrustmaster {
                public static final int X = 0;
                public static final int Y = 1;
                public static final int Z = 2;
                public static final int THROTTLE = 3;
            }       
        }
        public static class LogitechF310 {
            // ****when controller is in DirectInput mode (use slider on the back of the controller)
            public static final int A = 2;
            public static final int B = 3;
            public static final int X = 1;
            public static final int Y = 4;
            public static final int LB = 5;
            public static final int RB = 6;
            public static final int BACK = 9;
            public static final int START = 10;
            public static final int LEFT_PRESS = 7;
            public static final int RIGHT_PRESS = 8;
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
        public static final int ID_LEFT_FRONT = 30; //10
        public static final int ID_LEFT_REAR = 44; //11
        public static final int ID_RIGHT_FRONT = 43; //15
        public static final int ID_RIGHT_REAR = 45; //16

        public static final int INTAKE = 39;

        public static final int FIRST_FEEDER = 3;
        public static final int SECOND_FEEDER = 16;

        public static final int BOTTOM_SHOOTER = 36;
        public static final int TOP_SHOOTER = 33;

        public static final int ID_MAST = 38;
        public static final int ID_ARM = 37;
    }

    public static class Solenoids {
        public static final int INTAKE_SOL_FOR = 1;
        public static final int INTAKE_SOL_REV = 0;

        public static final int SPOON_SOL_FOR = 3;
        public static final int SPOON_SOL_REV = 2;

        public static final int HOOD_EXTEND = 4;
        public static final int HOOD_RETRACT = 5;
    }
    
    public static class AutoConstants {
        public static final int DIO_SWITCH_1 = 6;
        public static final int DIO_SWITCH_2 = 7;
        public static final int DIO_SWITCH_3 = 8;
        public static final int DIO_SWITCH_4 = 9;
    }
    public static class DriveConstants {
        public static final double LEFT_DEADZONE = 0.17; //0.15
        public static final double RIGHT_DEADZONE = 0.17;

        public static final int CURRENT_LIMIT = 40;
        // public static final int PEAK_CURRENT_LIMIT = 45;
        // public static final int PEAK_CURRENT_LIMIT_DURATION_MILLS = 100;

        public static final double DIAMETER = 4;
        public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
        public static final double GEAR_RATIO = 8.71 * 0.75;

        public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
        public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;

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
        // one degree is 0.239 inches *if the relationship is linear*
        public static final double TURN_180 = 43;
        public static final double TURN_135 = 25.7;
        public static final double TURN_90 = 21.5;
        public static final double TURN_70 = 16;
        public static final double TURN_15 = 6.5; // need to test this angle; should be able to head toward loading station ball during position 2 auto
        public static final double TURN_18 = 4.5;
        // distances
        public static final double AUTO_MARGIN = 0;
        public static final double HUB_TO_BALL = 95;
        public static final double TARMAC_TO_BALL = 55;
        public static final double TARMAC_TO_BALL_SHORT = 47; // short versions are for position 1 where the wall is close to ball
        public static final double BALL_TO_LINE = 23 - AUTO_MARGIN;
        public static final double BALL_TO_LINE_SHORT = 9 - AUTO_MARGIN; // short versions are for position 1 where the wall is close to ball
        public static final double TARMAC_TO_LINE = 20;
        public static final double TARMAC_TO_LOADING = 156;
	  
    }
    public static class ShooterConstants {
        public static final double kPBot = 0.0002;
        public static final double kIBot = 0.00000001;
        public static final double kDBot = 0.0565;
        public static final double kFFBot = 0.00021;

        public static final double kPTop = 0.0001;
        public static final double kITop = 0.00000001;
        public static final double kDTop = 0.0565;
        public static final double kFFTop = 0.00018;

        // speeds for top and bottom rollers
        // gear ratio is 18:32
        // speed on wheels should be greater than motor rpm
        // (wheel * 18) / 32 = motor
        public static final double TOP_SPEED = (2000 * 18) / 32; 
        public static final double BOT_SPEED = (2000 * 18) / 32; 

        public static final double HIGH_HUB_TOP = (6000 * 18) / 32;
        public static final double HIGH_HUB_BOT = (2200 * 18) / 32;

        public static final double TOP_BASKETBALL = (6000 * 18) / 32;
        public static final double BOT_BASKETBALL = (3850 * 18) / 32;

        public static final double LOW_HUB_TOP = (3400 * 18) / 32;
        public static final double LOW_HUB_BOT = (1700 * 18) / 32;

        // **** good tarmac shot: rollers should be at top:3650 and bot: 3466 ****
        public static final double TARMAC_TOP = (5000 * 18) / 32; //started with 5000
        public static final double TARMAC_BOT = (2500 * 32) / 18; // 2500
        // with waterbury pulley ratio
        // public static final double TARMAC_TOP = (3650 * 18) / 32; //1237; // wheel speed = 2200; //3650
        // public static final double TARMAC_BOT = (3350 * 18) / 32; //2261; // wheel speed = 4020;

        public static final double TARMAC_ATOP = (3350 * 18) / 32; //1237; // wheel speed = 2200; //3650
        public static final double TARMAC_ABOT = (3325 * 18) / 32; //2261; // wheel speed = 4020;

        // 3650, 3075
        // 3350, 3325
        // 6000, 2500

        // increase bottom for more loft, decrease top to make shorter & less backspin
        public static final double TARMAC_TOP_XSHORT = (3400 * 18) / 32; //1237; // wheel speed = 2200; //3650
        public static final double TARMAC_BOT_XSHORT = (3600 * 18) / 32; //2261; // wheel speed = 4020;

        public static final double TARMAC_TOP_SHORT = (3500 * 18) / 32; //1237; // wheel speed = 2200; //3650
        public static final double TARMAC_BOT_SHORT = (3600 * 18) / 32; //2261; // wheel speed = 4020;

        public static final double TARMAC_TOP_LONG = (3670 * 18) / 32; //1237; // wheel speed = 2200; //3650
        public static final double TARMAC_BOT_LONG = (3520 * 18) / 32; //2261; // wheel speed = 4020;

        // ***increase top and decrease bottom for more height and less depth
        public static final double LAUNCH_PAD_TOP = (7450 * 18) / 32; //6050
        public static final double LAUNCH_PAD_BOT = (3450 * 18) / 32; //2850
        // 7510, 3500 was too long
    }
    public static class IntakeConstants {
        public static final int DIO_INTAKE_EYE = 5;
        public static final int DIO_FEEDER_EYE = 0;

        public static final double FORWARD_SPEED = 0.85;
        // 0.6 voltage = 930 RPM
        // 0.8 voltage = 1280 RPM
        public static final double REVERSE_SPEED = -0.5;

        public static final double FIRST_FEED_SPEED = 0.3;
        public static final double SECOND_FEED_SPEED = 0.3;
    }
    public static class ColorSensorConstants {
        public static final int DIST = 300;
    }
    public static class ClimberConstants {
        public static final double MAST_EX_SPEED = 0.3;
        public static final double MAST_RE_SPEED = 0.3;
        public static final double ARM_EX_SPEED = 0.5;
        public static final double ARM_RE_SPEED = 0.5;

        public static double kParm = 0;
        public static double kIarm = 0;
        public static double kDarm = 0;
        public static double kFarm = 0; // mooooooooo

        public static double kPmast = 0.025;
        public static double kImast = 0;
        public static double kDmast = 0;
        public static double kFmast = 0;

        public static double armREV_TO_IN = 1.64;
        public static double armIN_TO_REV = 0.6097;

        public static double mastREV_TO_IN = 0.5;
        public static double mastIN_TO_REV = 2;

        public static double climberMIN_OUTPUT = -1;
        public static double climberMAX_OUTPUT = 1;

        public static final double armMARGIN = 2;
        public static final double armDISTANCE = 18;

        public static final double mastMARGIN = 2;
        public static final double MAST_EXT_RET_DIST = 50;

        public static final int DIO_MAST_RETURN = 3;
        public static final int DIO_MAST_EXTEND = 4;

        public static final int DIO_ARM_RETRACT = 1;
        public static final int DIO_ARM_EXTEND = 2;

        public static final int CURRENT_LIMIT_CLIMB = 40;

    }

}
