// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

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
import frc.robot.commands.SpoonExtend;
import frc.robot.commands.SpoonRetract;
import frc.robot.commands.GetColorSensor;
import frc.robot.commands.Auto.TestCmdGroup;
import frc.robot.commands.Drive.DashboardPID;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.DriveWithPID;
import frc.robot.commands.Drive.WPIpid;
import frc.robot.commands.Drive.TurnWithPID;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.IntakeExtendAndRetract;
import frc.robot.commands.Intake.IntakeRetract;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Shooter.RawShoot;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.subsystems.PneumaticTest;

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
    // private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final ColorSensor colorSensor = new ColorSensor();
    private final PneumaticTest pneumaticTest = new PneumaticTest();
    // private final LoadingSpoon loadingSpoon = new LoadingSpoon();
 
  // **COMMANDS**
    // *AUTO
    private final TestCmdGroup testCmdGroup = new TestCmdGroup(drive);
    // *DRIVE
    private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
    private final DriveWithPID driveWithPID = new DriveWithPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final DashboardPID dashboardPID = new DashboardPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final TurnWithPID turnWithPID = new TurnWithPID(drive, DriveConstants.TURN_DISTANCE, DriveConstants.MARGIN);
    private final WPIpid newPID = new WPIpid(drive);
    // *SHOOTER
    // private final Shoot shoot = new Shoot(shooter, ShooterConstants.LOW_HUB_LARGE, Constants.ShooterConstants.LOW_HUB_SMALL);
    // private final RawShoot rawShoot = new RawShoot(shooter, ShooterConstants.BOT_SPEED, ShooterConstants.TOP_SPEED);
    // *INTAKE
    // private final SetIntakeSpeed intakeForward = new SetIntakeSpeed(intake, IntakeConstants.FORWARD_SPEED);
    // private final SetIntakeSpeed intakeReverse = new SetIntakeSpeed(intake, IntakeConstants.REVERSE_SPEED);
    private final IntakeExtendAndRetract intakeExtendAndRetract = new IntakeExtendAndRetract(intake);
    private final IntakeExtend intakeExtend = new IntakeExtend(intake);
    private final IntakeRetract intakeRetract = new IntakeRetract(intake);
    // *LOADING SPOON
    // private final SpoonExtend spoonExtend = new SpoonExtend(loadingSpoon);
    // private final SpoonRetract spoonRetract = new SpoonRetract(loadingSpoon);
    // private final SolenoidForward solenoidForward = new SolenoidForward(pneumaticTest);
    // private final SolenoidReverse solenoidReverse = new SolenoidReverse(pneumaticTest);
    // *COLOR SENSOR
    // private final GetColorSensor getColorSensor = new GetColorSensor(colorSensor);
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
    JoystickButton rightMiddle = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_MIDDLE);
    JoystickButton rightStickLeft = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_LEFT);
    JoystickButton rightStickRight = new JoystickButton(rightStick, ControllerConstants.Thrustmaster.BUTTON_RIGHT);

    // ASSIGN BUTTONS TO COMMANDS
    // a.whileHeld(dashboardPID);
    // b.whenPressed(shoot);
    x.whenPressed(driveWithPID);
    a.whileHeld(turnWithPID);
    b.whileHeld(dashboardPID);
    y.whileHeld(newPID);
    // rightStickLeft.whenPressed(solenoidForward);
    // rightStickRight.whenPressed(solenoidReverse);
    // y.whileHeld(getColorSensor);
    // rb.whileHeld(intakeForward);
    // lb.whileHeld(intakeReverse);
    // b.whenPressed(intakeExtendAndRetract);
    // y.whenPressed(intakeExtend);
    // a.whenPressed(intakeRetract);
    // rb.whileHeld(intakeForward);
    // lb.whileHeld(intakeReverse);
    // y.whenPressed(spoonExtend);
    // a.whenPressed(spoonRetract);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return testCmdGroup;
  }
}
