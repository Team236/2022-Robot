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
import frc.robot.commands.Auto.TriplePosition1;
import frc.robot.commands.Auto.DoubleTarmac1;
import frc.robot.commands.Auto.DoubleTarmac2;
import frc.robot.commands.Auto.ShootMoveShoot;
import frc.robot.commands.Auto.TriplePosition2;
import frc.robot.commands.Auto.ExitTarmac;
import frc.robot.commands.Auto.ExtendedTriple1;
import frc.robot.commands.Auto.QuadruplePosition1;
import frc.robot.commands.Climber.ArmPID;
import frc.robot.commands.Climber.ArmWithAxis;
import frc.robot.commands.Climber.MastPID;
import frc.robot.commands.Climber.MastSetHeight;
import frc.robot.commands.Climber.MastWithAxis;
import frc.robot.commands.Drive.DashboardPID;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Drive.WPI_Turn_PID;
import frc.robot.commands.Hood.HoodExtendAndRetract;
import frc.robot.commands.Intake.IntakeExtendAndRetract;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.NewIntakeForward;
import frc.robot.commands.Intake.SetFeedSpeed;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Shooter.FeedAndShoot;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Mast;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Drive.AngleAndDistLL;
import frc.robot.commands.Drive.AnglewithLL;
import frc.robot.commands.Drive.DistancewithLL;
import frc.robot.commands.Drive.DriveForward;

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
    private final Hood hood = new Hood();
    private final Mast mast = new Mast();
    // private final Arm arm = new Arm();

  // **COMMANDS**
    // *AUTO
    private final ExitTarmac exitTarmac = new ExitTarmac(drive);
    private final ShootMoveShoot shootMoveShoot = new ShootMoveShoot(drive, shooter, hood, intake);
    private final DoubleHubShot doubleHubShot = new DoubleHubShot(drive, shooter, hood, intake);
    private final DoubleTarmac1 doubleTarmac1 = new DoubleTarmac1(drive, shooter, hood, intake);
    private final DoubleTarmac2 doubleTarmac2 = new DoubleTarmac2(drive, intake, shooter, hood);
    private final TriplePosition1 triplePosition1 = new TriplePosition1(intake, drive, hood, shooter);
    private final TriplePosition2 triplePosition2 = new TriplePosition2(drive, intake, shooter, hood);
    private final QuadruplePosition1 quadruplePosition1 = new QuadruplePosition1(drive, intake, hood, shooter);
    private final ExtendedTriple1 extendedTriple1 = new ExtendedTriple1(intake, drive, hood, shooter);

    // *AUTO SWITCHES
    // private static DigitalInput autoSwitch1, autoSwitch2, autoSwitch3, autoSwitch4;
    private static DigitalInput autoSwitch1 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_1);
    private static DigitalInput autoSwitch2 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_2);
    private static DigitalInput autoSwitch3 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_3);
    private static DigitalInput autoSwitch4 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_4);
    // *DRIVE
    private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
    private final DashboardPID dashboardPID = new DashboardPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final DriveForward driveForward = new DriveForward(drive, 0.3);
    // private final TurnWithPID turnWithPID = new TurnWithPID(drive, DriveConstants.TURN_DISTANCE, DriveConstants.MARGIN);
    // *SHOOTER
    
    private final FeedAndShoot feedAndShootTarmac = new  FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP);
    private final FeedAndShoot feedAndShootLow = new FeedAndShoot(intake, shooter, hood, ShooterConstants.LOW_HUB_BOT, ShooterConstants.LOW_HUB_TOP);
    private final FeedAndShoot feedAndShootSafe = new FeedAndShoot(intake, shooter, hood, ShooterConstants.LAUNCH_PAD_BOT, ShooterConstants.LAUNCH_PAD_TOP);
    // *LIMELIGHT
    private final AnglewithLL anglewithLL = new AnglewithLL(drive);
    private final DistancewithLL distancewithLL = new DistancewithLL(drive);
    private final AngleAndDistLL angleAndDistLL = new AngleAndDistLL(drive);
    // private final TrackBall trackBall = new TrackBall(drive);
    // *HOOD
    private final HoodExtendAndRetract hoodExtendAndRetract = new HoodExtendAndRetract(hood);
    // *INTAKE
    private final IntakeExtendAndRetract intakeExtendAndRetract = new IntakeExtendAndRetract(intake);
    private final IntakeReverse intakeReverse = new IntakeReverse(intake, IntakeConstants.REVERSE_SPEED);
    private final SetIntakeSpeed rawIntakeForward = new SetIntakeSpeed(intake, IntakeConstants.FORWARD_SPEED);
    private final SetFeedSpeed firstFeed = new SetFeedSpeed(intake, IntakeConstants.FIRST_FEED_SPEED);
    private final NewIntakeForward newIntakeForward = new NewIntakeForward(intake);
    private final SetFeedSpeed reverseFeed = new SetFeedSpeed(intake, -IntakeConstants.FIRST_FEED_SPEED);
    // *CLIMBER
    private final MastWithAxis mastWithAxis = new MastWithAxis(mast, controller);
    // private final ArmWithAxis armWithAxis = new ArmWithAxis(arm, controller);
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
    a.whileActiveOnce(new MastPID(mast, -140, 1)); //retracts climber to climb on mid rung
    y.whileActiveOnce(new MastPID(mast, 169.5, 1)); //raises climber for mid rung
    x.whileActiveOnce(new MastPID(mast, ClimberConstants.PIT_MAST, 1)); //pit button--preps climber for match
    lb.whenPressed(hoodExtendAndRetract);
    rb.whenPressed(intakeExtendAndRetract);
    leftPress.whileHeld(anglewithLL);
    rightPress.whileHeld(distancewithLL);
    start.whileHeld(mastWithAxis); // hold down start while using left joystick on controller

    // *LEFT STICK
    leftTrigger.whileHeld(feedAndShootTarmac);
    leftMiddle.whileHeld(feedAndShootSafe);
    // leftStickLeft.whileHeld(trackBall);
    leftStickRight.whileHeld(feedAndShootLow);
    extraL1.whileHeld(new WPI_PID(drive, 50));
    extraL2.whileHeld(new Shoot(shooter, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP));
    extraL8.whenPressed(hoodExtendAndRetract);
    
    // *RIGHT STICK
    rightTrigger.whileActiveOnce(newIntakeForward);
    rightMiddle.whileActiveOnce(intakeReverse);
    rightStickLeft.whileActiveOnce(new WPI_Turn_PID(drive, 169)); //turns clockwise/to the left
    rightStickRight.whileActiveOnce(new WPI_Turn_PID(drive, 91));
    extraR1.whileHeld(reverseFeed);
    extraR2.whileHeld(firstFeed);
    extraR4.whenPressed(intakeExtendAndRetract);
    extraR7.whileHeld(anglewithLL);
    extraR8.whileHeld(distancewithLL);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // an auto switch is "on" if the autoSwitch.get is false
    if (!autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && !autoSwitch4.get()) {
      return exitTarmac;
    } else if (!autoSwitch1.get() && !autoSwitch3.get()) {
      return quadruplePosition1;
    } else if (!autoSwitch1.get() && !autoSwitch4.get()) {
      return extendedTriple1;
    } else if (!autoSwitch1.get()) {
      return doubleTarmac1;
    } else if (!autoSwitch2.get()) {
      return doubleTarmac2;
    } else if (!autoSwitch3.get()) {
      return triplePosition1;
    } else if (!autoSwitch4.get()) {
      return triplePosition2;
    } else {
      return doubleTarmac1;
    }
  }
}
