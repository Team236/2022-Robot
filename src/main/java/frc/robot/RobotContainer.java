// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Auto.DoubleHubShot;
import frc.robot.commands.Auto.DoubleTarmac3;
import frc.robot.commands.Auto.DoubleTarmac1;
import frc.robot.commands.Auto.DoubleTarmac2;
import frc.robot.commands.Auto.ShootMoveShoot;
import frc.robot.commands.Auto.ExitTarmac;
import frc.robot.commands.Climber.ArmPID;
import frc.robot.commands.Climber.ClimbSequence;
import frc.robot.commands.Climber.MastPID;
import frc.robot.commands.Climber.MastWithAxis;
import frc.robot.commands.Drive.DashboardPID;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.DriveWithPID;
import frc.robot.commands.Drive.TurnWithPID;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Drive.WPI_Turn_PID;
import frc.robot.commands.Hood.HoodExtend;
import frc.robot.commands.Hood.HoodExtendAndRetract;
import frc.robot.commands.Hood.HoodRetract;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.IntakeExtendAndRetract;
import frc.robot.commands.Intake.IntakeRetract;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Intake.IntakeForward;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShootWithLL;
import frc.robot.commands.Shooter.SpoonAndShoot;
import frc.robot.commands.Spoon.SpoonCmdGroup;
import frc.robot.commands.Spoon.SpoonExtend;
import frc.robot.commands.Spoon.SpoonExtendAndRetract;
import frc.robot.commands.Spoon.SpoonRetract;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.commands.Drive.AngleAndDistLL;
import frc.robot.commands.Drive.AnglewithLL;
import frc.robot.commands.Drive.DistancewithLL;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // **JOYSTICKS**
  Joystick controller = new Joystick(Constants.ControllerConstants.USB_CONTROLLER);
  Joystick leftStick = new Joystick(Constants.ControllerConstants.USB_LEFT_STICK);
  Joystick rightStick = new Joystick(Constants.ControllerConstants.USB_RIGHT_STICK);

  // **SUBSYSTEMS**
    private final Drive drive = new Drive();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final LoadingSpoon loadingSpoon = new LoadingSpoon();
    private final Hood hood = new Hood();
    private final Climber climber = new Climber();
 
  // **COMMANDS**
    // *AUTO
    private final ExitTarmac exitTarmac = new ExitTarmac(drive);
    private final DoubleHubShot doubleHubShot = new DoubleHubShot(drive, shooter, hood, loadingSpoon, intake);
    private final DoubleTarmac1 doubleTarmac1 = new DoubleTarmac1(drive, shooter, hood, loadingSpoon, intake);
    private final DoubleTarmac2 doubleTarmac2 = new DoubleTarmac2(drive, intake, loadingSpoon, shooter, hood);
    private final DoubleTarmac3 doubleTarmac3 = new DoubleTarmac3(intake, drive, hood, loadingSpoon, shooter);
    private final ShootMoveShoot shootMoveShoot = new ShootMoveShoot(drive, shooter, hood, loadingSpoon, intake);

    // *AUTO SWITCHES
    // private static DigitalInput autoSwitch1, autoSwitch2, autoSwitch3, autoSwitch4;
    private static DigitalInput autoSwitch1 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_1);
    private static DigitalInput autoSwitch2 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_2);
    private static DigitalInput autoSwitch3 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_3);
    private static DigitalInput autoSwitch4 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_4);
    // *DRIVE
    private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
    private final DriveWithPID driveWithPID = new DriveWithPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final DashboardPID dashboardPID = new DashboardPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    // private final TurnWithPID turnWithPID = new TurnWithPID(drive, DriveConstants.TURN_DISTANCE, DriveConstants.MARGIN);
    // *SHOOTER
    private final Shoot shoot = new Shoot(shooter, ShooterConstants.HIGH_HUB_BOT, Constants.ShooterConstants.HIGH_HUB_TOP);
    private final SpoonAndShoot spoonAndShootHigh = new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.HIGH_HUB_BOT, ShooterConstants.HIGH_HUB_TOP);
    private final SpoonAndShoot spoonAndShootLow = new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.LOW_HUB_BOT, ShooterConstants.LOW_HUB_TOP);
    private final SpoonAndShoot spoonAndShootTarmac = new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP);
    private final SpoonAndShoot spoonAndShootLaunchPad = new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.LAUNCH_PAD_BOT, ShooterConstants.LAUNCH_PAD_TOP);
    private final ShootWithLL shootHighWithLL = new ShootWithLL(drive, loadingSpoon, shooter, hood, ShooterConstants.HIGH_HUB_BOT, ShooterConstants.HIGH_HUB_TOP);
    private final ShootWithLL shootTarmacWLL = new ShootWithLL(drive, loadingSpoon, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP);
    // *LIMELIGHT
    private final AnglewithLL anglewithLL = new AnglewithLL(drive);
    private final DistancewithLL distancewithLL = new DistancewithLL(drive);
    private final AngleAndDistLL angleAndDistLL = new AngleAndDistLL(drive);
    // *HOOD
    private final HoodExtendAndRetract hoodExtendAndRetract = new HoodExtendAndRetract(hood);
    private final HoodExtend hoodExtend = new HoodExtend(hood);
    private final HoodRetract hoodRetract = new HoodRetract(hood);
    // *INTAKE
    private final IntakeExtendAndRetract intakeExtendAndRetract = new IntakeExtendAndRetract(intake);
    private final IntakeExtend intakeExtend = new IntakeExtend(intake);
    private final IntakeRetract intakeRetract = new IntakeRetract(intake);
    private final IntakeForward intakeForward = new IntakeForward(intake, IntakeConstants.FORWARD_SPEED);
    private final IntakeReverse intakeReverse = new IntakeReverse(intake, IntakeConstants.REVERSE_SPEED);
    private final SetIntakeSpeed rawIntakeForward = new SetIntakeSpeed(intake, IntakeConstants.FORWARD_SPEED);
    // *LOADING SPOON
    private final SpoonExtend spoonExtend = new SpoonExtend(loadingSpoon);
    private final SpoonRetract spoonRetract = new SpoonRetract(loadingSpoon);
    private final SpoonExtendAndRetract spoonExtendAndRetract = new SpoonExtendAndRetract(loadingSpoon);
    private final SpoonCmdGroup extendWaitRetract = new SpoonCmdGroup(loadingSpoon);
    // *CLIMBER
    private final ArmPID extendArm = new ArmPID(climber, ClimberConstants.armDISTANCE, ClimberConstants.armMARGIN);
    private final MastPID raiseMast = new MastPID(climber, ClimberConstants.MAST_EXT_RET_DIST, ClimberConstants.mastMARGIN);
    private final ClimbSequence climbSequence = new ClimbSequence(climber);
    private final MastWithAxis mastWithAxis = new MastWithAxis(climber, controller);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    drive.setDefaultCommand(driveWithJoysticks);

    // Configure the button bindings
    configureButtonBindings();

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // CREATE BUTTONS
    // *CONTROLLER
    JoystickButton x = new JoystickButton(controller, ControllerConstants.LogitechF310.X);
    JoystickButton a = new JoystickButton(controller, ControllerConstants.LogitechF310.A);
    JoystickButton b = new JoystickButton(controller, ControllerConstants.LogitechF310.B);
    JoystickButton y = new JoystickButton(controller, ControllerConstants.LogitechF310.Y);
    JoystickButton lb = new JoystickButton(controller, ControllerConstants.LogitechF310.LB);
    JoystickButton rb = new JoystickButton(controller, ControllerConstants.LogitechF310.RB);
    JoystickButton back = new JoystickButton(controller, ControllerConstants.LogitechF310.BACK);
    JoystickButton start = new JoystickButton(controller, ControllerConstants.LogitechF310.START);
    JoystickButton leftPress = new JoystickButton(controller, ControllerConstants.LogitechF310.LEFT_PRESS);
    JoystickButton rightPress = new JoystickButton(controller, ControllerConstants.LogitechF310.RIGHT_PRESS);

    // *LEFT STICK
    JoystickButton leftTrigger = new JoystickButton(leftStick,ControllerConstants.Thrustmaster.TRIGGER);
    JoystickButton leftMiddle = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton leftStickLeft = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton leftStickRight = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);
    JoystickButton extraL1 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_1);
    JoystickButton extraL2 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_2);
    JoystickButton extraL3 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_3);
    JoystickButton extraL4 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_4);
    JoystickButton extraL5 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_5);
    JoystickButton extraL6 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_6);
    JoystickButton extraL7 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_7);
    JoystickButton extraL8 = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.RIGHT_BASE_8);
    
    // *RIGHT STICK
    JoystickButton rightTrigger = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.TRIGGER); 
    JoystickButton rightMiddle = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton rightStickLeft = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton rightStickRight = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);
    JoystickButton extraR1 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_1);
    JoystickButton extraR2 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_2);
    JoystickButton extraR3 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_3);
    JoystickButton extraR4 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.LEFT_BASE_4);
    JoystickButton extraR5 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_5);
    JoystickButton extraR6 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_6);
    JoystickButton extraR7 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_7);
    JoystickButton extraR8 = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_8);

    // ASSIGN BUTTONS TO COMMANDS

    // **if whenPressed is used for PID commands, you cannot drive with joysticks after!!
    // *CONTROLLER
    x.whenPressed(new ArmPID(climber, ClimberConstants.armDISTANCE, ClimberConstants.armMARGIN));
    b.whileActiveOnce(new ClimbSequence(climber));
    y.whenPressed(new MastPID(climber, ClimberConstants.MAST_EXT_RET_DIST, ClimberConstants.mastMARGIN));
    a.whenPressed(new MastPID(climber, -ClimberConstants.MAST_EXT_RET_DIST, ClimberConstants.mastMARGIN));
    lb.whenPressed(hoodExtendAndRetract);
    rb.whenPressed(intakeExtendAndRetract);
    leftPress.whileActiveOnce(anglewithLL);
    rightPress.whileActiveOnce(distancewithLL);
    back.whenPressed(spoonExtendAndRetract);
    start.whileActiveOnce(new WPI_PID(drive, 27)); // position robot so camera barely sees white line, use this button to move out of tarmac, then shoot
    // *LEFT STICK
    leftTrigger.whileActiveOnce(new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP)); //spoonAndShootTarmac
    leftMiddle.whileActiveOnce(new WPI_Turn_PID(drive, DriveConstants.TURN_180));
    leftStickLeft.whileActiveOnce(new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.HIGH_HUB_BOT, ShooterConstants.HIGH_HUB_TOP)); //spoonAndShootHigh
    leftStickRight.whileActiveOnce(new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.LOW_HUB_BOT, ShooterConstants.LOW_HUB_TOP)); //spoonAndShootLow
    extraL1.whileActiveOnce(new WPI_PID(drive, 17));
    extraL5.whenPressed(hoodExtend);
    extraL6.whenPressed(hoodRetract);
    extraL7.whenPressed(spoonExtend);
    extraL8.whenPressed(spoonRetract);
    // *RIGHT STICK
    rightTrigger.whileActiveOnce(intakeForward);
    rightMiddle.whileActiveOnce(intakeReverse);
    rightStickLeft.whileHeld(rawIntakeForward);
    rightStickRight.whileActiveOnce(new WPI_Turn_PID(drive, DriveConstants.TURN_90));
    extraR3.whenPressed(intakeExtend);
    extraR4.whenPressed(intakeRetract);
    extraR7.whileActiveOnce(anglewithLL);
    extraR8.whileActiveOnce(distancewithLL);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (!autoSwitch1.get()) {
      return doubleTarmac1;
    } else if (!autoSwitch2.get()) {
      return doubleTarmac2;
    } else if (!autoSwitch3.get()) {
      return doubleTarmac3;
    } else if (!autoSwitch4.get()) {
      return doubleHubShot;
    } else {
      return exitTarmac;
    }

    // return doubleTarmac1;
  }
}
