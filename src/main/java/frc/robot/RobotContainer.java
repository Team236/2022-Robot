// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ControllerConstants.LogitechF310;
import frc.robot.Constants.ControllerConstants.Thrustmaster;
import frc.robot.commands.Auto.DoubleHubShot;
import frc.robot.commands.Auto.DoubleTarmacLineShot;
import frc.robot.commands.Auto.ShootMoveShoot;
import frc.robot.commands.Auto.TestCmdGroup;
import frc.robot.commands.Climber.ArmPID;
import frc.robot.commands.Climber.ClimbSequence;
import frc.robot.commands.Climber.MastPIDUp;
import frc.robot.commands.Drive.DashboardPID;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.DriveWithPID;
import frc.robot.commands.Drive.TurnWithPID;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Drive.WPI_Turn_PID;
import frc.robot.commands.Hood.HoodExtendAndRetract;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.IntakeExtendAndRetract;
import frc.robot.commands.Intake.IntakeRetract;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Intake.IntakeForward;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShootWithLL;
import frc.robot.commands.Shooter.SpoonAndShoot;
import frc.robot.commands.Spoon.ExtendWaitRetract;
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
    private final TestCmdGroup testCmdGroup = new TestCmdGroup(drive);
    private final DoubleHubShot doubleHubShot = new DoubleHubShot(drive, shooter, loadingSpoon, intake);
    private final DoubleTarmacLineShot doubleTarmacLineShot = new DoubleTarmacLineShot(drive, shooter, loadingSpoon, intake);
    private final ShootMoveShoot shootMoveShoot = new ShootMoveShoot(drive, shooter, loadingSpoon, intake);
    
    // *DRIVE
    private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
    private final DriveWithPID driveWithPID = new DriveWithPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final DashboardPID dashboardPID = new DashboardPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    // private final TurnWithPID turnWithPID = new TurnWithPID(drive, DriveConstants.TURN_DISTANCE, DriveConstants.MARGIN);
    // *SHOOTER
    // private final Shoot shoot = new Shoot(shooter, ShooterConstants.HIGH_HUB_LARGE, Constants.ShooterConstants.HIGH_HUB_SMALL);
    private final SpoonAndShoot spoonAndShootHigh = new SpoonAndShoot(loadingSpoon, shooter, ShooterConstants.HIGH_HUB_BOT, ShooterConstants.HIGH_HUB_TOP);
    private final SpoonAndShoot spoonAndShootLow = new SpoonAndShoot(loadingSpoon, shooter, ShooterConstants.LOW_HUB_BOT, ShooterConstants.LOW_HUB_TOP);
    private final SpoonAndShoot spoonAndShootTarmac = new SpoonAndShoot(loadingSpoon, shooter, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP);
    private final SpoonAndShoot spoonAndShootLaunchPad = new SpoonAndShoot(loadingSpoon, shooter, ShooterConstants.LAUNCH_PAD_BOT, ShooterConstants.LAUNCH_PAD_TOP);
    private final ShootWithLL shootHighWithLL = new ShootWithLL(drive, loadingSpoon, shooter, ShooterConstants.HIGH_HUB_BOT, ShooterConstants.HIGH_HUB_TOP);
    // *LIMELIGHT
    private final AnglewithLL anglewithLL = new AnglewithLL(drive);
    private final DistancewithLL distancewithLL = new DistancewithLL(drive);
    private final AngleAndDistLL angleAndDistLL = new AngleAndDistLL(drive);
    // *HOOD
    private final HoodExtendAndRetract hoodExtendAndRetract = new HoodExtendAndRetract(hood);
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
    private final ExtendWaitRetract extendWaitRetract = new ExtendWaitRetract(loadingSpoon);
    // *CLIMBER
    private final ArmPID extendArm = new ArmPID(climber, 6, 1);
    private final MastPIDUp raiseMast = new MastPIDUp(climber, 6, 1);
    private final ClimbSequence climbSequence = new ClimbSequence(climber);

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

    // *RIGHT STICK
    JoystickButton rightTrigger = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.TRIGGER); 
    JoystickButton rightMiddle = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton rightStickLeft = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton rightStickRight = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);
    JoystickButton thridExtraButton = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_2);
    JoystickButton fourthExtraButton = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.RIGHT_BASE_1);

    // *LEFT STICK
    JoystickButton leftTrigger = new JoystickButton(leftStick,ControllerConstants.Thrustmaster.TRIGGER);
    JoystickButton leftMiddle = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton leftStickLeft = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton leftStickRight = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);
    JoystickButton firstExtraButton = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_1);
    JoystickButton secondExtraButton = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.LEFT_BASE_2);

    // ASSIGN BUTTONS TO COMMANDS
    // **if whenPressed is used for PID commands, you cannot drive with joysticks after!!
    // *CONTROLLER
    x.whenPressed(new ArmPID(climber, ClimberConstants.armDISTANCE, ClimberConstants.armMARGIN));
    b.whileActiveOnce(new ClimbSequence(climber));
    y.whenPressed(new MastPIDUp(climber, ClimberConstants.mastDISTANCE, ClimberConstants.mastMARGIN));
    lb.whenPressed(hoodExtendAndRetract);
    rb.whenPressed(intakeExtendAndRetract);
    // *RIGHT STICK
    rightTrigger.whileActiveOnce(intakeForward);
    rightMiddle.whileActiveOnce(intakeReverse);
    rightStickLeft.whileHeld(rawIntakeForward);
    // *LEFT STICK
    leftTrigger.whileActiveOnce(spoonAndShootTarmac);
    leftMiddle.whileActiveOnce(new WPI_Turn_PID(drive, DriveConstants.TURN_180));
    leftStickLeft.whileActiveOnce(spoonAndShootHigh);
    leftStickRight.whileActiveOnce(spoonAndShootLow);
    secondExtraButton.whenPressed(spoonRetract);

    // *TESTING
    // rightMiddle.whileActiveOnce(new WPI_Turn_PID(drive, DriveConstants.TURN_180));
    // b.whenPressed(testCmdGroup);
    // x.whileActiveOnce(new WPI_PID(drive, DriveConstants.DISTANCE));
    // y.whileHeld(new Shoot(shooter, ShooterConstants.LOW_HUB_BOT, ShooterConstants.LOW_HUB_TOP));
    // rb.whenPressed(spoonExtend);
    // lb.whenPressed(spoonRetract);
    // rightTrigger.whileHeld(intakeForward);
    // rightTrigger.whileActiveOnce(intakeForward);
    // leftTrigger.whileActiveOnce(angleAndDistLL);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return testCmdGroup;
  }
}
