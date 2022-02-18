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
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ControllerConstants.LogitechF310;
import frc.robot.Constants.ControllerConstants.Thrustmaster;
import frc.robot.commands.Auto.TestCmdGroup;
import frc.robot.commands.Drive.AnotherOne;
import frc.robot.commands.Drive.DashboardPID;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.DriveWithPID;
import frc.robot.commands.Drive.TurnWithPID;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.IntakeExtendAndRetract;
import frc.robot.commands.Intake.IntakeRetract;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Intake.IntakeForward;
import frc.robot.commands.Shooter.ParallelCmdGroup;
import frc.robot.commands.Shooter.RawShoot;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.SpoonAndShoot;
import frc.robot.commands.Spoon.ExtendWaitRetract;
import frc.robot.commands.Spoon.SpoonExtend;
import frc.robot.commands.Spoon.SpoonExtendAndRetract;
import frc.robot.commands.Spoon.SpoonExtendThenRetract;
import frc.robot.commands.Spoon.SpoonRetract;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LoadingSpoon;

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
  // Counter ballCounter = new Counter();

  // **SUBSYSTEMS**
    private final Drive drive = new Drive();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    // private final ColorSensor colorSensor = new ColorSensor();
    private final LoadingSpoon loadingSpoon = new LoadingSpoon();
 
  // **COMMANDS**
    // *AUTO
    private final TestCmdGroup testCmdGroup = new TestCmdGroup(drive);
    
    // *DRIVE
    private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
    private final DriveWithPID driveWithPID = new DriveWithPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final DashboardPID dashboardPID = new DashboardPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final TurnWithPID turnWithPID = new TurnWithPID(drive, DriveConstants.TURN_DISTANCE, DriveConstants.MARGIN);
    private final AnotherOne anothaPID = new AnotherOne(drive);
    // *SHOOTER
    private final Shoot shoot = new Shoot(shooter, ShooterConstants.HIGH_HUB_LARGE, Constants.ShooterConstants.HIGH_HUB_SMALL);
    // private final RawShoot rawShoot = new RawShoot(shooter, ShooterConstants.BOT_SPEED, ShooterConstants.TOP_SPEED);
    private final SpoonAndShoot spoonAndShoot = new SpoonAndShoot(loadingSpoon, shooter);
    private final ParallelCmdGroup parallelCmdGroup = new ParallelCmdGroup(loadingSpoon, shooter);
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
    private final SpoonExtendThenRetract spoonExtendThenRetract = new SpoonExtendThenRetract(loadingSpoon);
    private final ExtendWaitRetract extendWaitRetract = new ExtendWaitRetract(loadingSpoon);
    // *COLOR SENSOR
    // private final GetColorSensor getColorSensor = new GetColorSensor(colorSensor);
    

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
    JoystickButton a = new JoystickButton(controller, ControllerConstants.LogitechF310.A);
    JoystickButton b = new JoystickButton(controller, ControllerConstants.LogitechF310.B);
    JoystickButton x = new JoystickButton(controller, ControllerConstants.LogitechF310.X);
    JoystickButton y = new JoystickButton(controller, ControllerConstants.LogitechF310.Y);
    JoystickButton lb = new JoystickButton(controller, ControllerConstants.LogitechF310.LB);
    JoystickButton rb = new JoystickButton(controller, ControllerConstants.LogitechF310.RB);
    JoystickButton back = new JoystickButton(controller, ControllerConstants.LogitechF310.BACK);
    JoystickButton start = new JoystickButton(controller, ControllerConstants.LogitechF310.START);
    JoystickButton leftPress = new JoystickButton(controller, ControllerConstants.LogitechF310.LEFT_PRESS);
    JoystickButton rightPress = new JoystickButton(controller, ControllerConstants.LogitechF310.RIGHT_PRESS);

    JoystickButton rightTrigger = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.TRIGGER);
    JoystickButton leftTrigger = new JoystickButton(leftStick, ControllerConstants.Thrustmaster.TRIGGER);
    JoystickButton rightMiddle = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton rightStickLeft = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton rightStickRight = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);

    // ASSIGN BUTTONS TO COMMANDS
    // a.whenPressed(spoonRetract);
    b.whenPressed(shoot);
    x.whenPressed(spoonAndShoot);
    // y.whenPressed(spoonAndShoot);
    rb.whenPressed(spoonExtend);
    lb.whenPressed(spoonRetract);
    rightTrigger.whileHeld(intakeForward);
    leftTrigger.whileHeld(rawIntakeForward);
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
