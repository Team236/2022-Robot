// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MainControllerConstants;
import frc.robot.Constants.MainControllerConstants.Thrustmaster;
import frc.robot.commands.DashboardPID;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithPID;
import frc.robot.commands.SolenoidForward;
import frc.robot.commands.SolenoidReverse;
import frc.robot.commands.Auto.DriveDistance;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // **SUBSYSTEMS**
    private final Drive drive = new Drive();

  // **JOYSTICKS**
   Joystick controller = new Joystick(ControllerConstants.USB_CONTROLLER);
   Joystick leftStick = new Joystick(ControllerConstants.USB_LEFT_STICK);
   Joystick rightStick = new Joystick(ControllerConstants.USB_RIGHT_STICK);

  // **COMMANDS**
     
    // AUTO
    private final DriveDistance driveDistance = new DriveDistance(drive);
    // DRIVE
    private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
    private final DriveWithPID driveWithPID = new DriveWithPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    private final DashboardPID dashboardPID = new DashboardPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
    // PNEUMATICS
    private final SolenoidForward solenoidForward = new SolenoidForward();
    private final SolenoidReverse solenoidReverse = new SolenoidReverse();

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

    JoystickButton leftTrigger = new JoystickButton(leftStick, MainControllerConstants.Thrustmaster.joyTRIGGER);
    leftTrigger.whenPressed(driveWithPID);

    JoystickButton rightTrigger = new JoystickButton(rightStick, MainControllerConstants.Thrustmaster.joyTRIGGER);
    rightTrigger.whileHeld(dashboardPID);

    JoystickButton leftMiddle = new JoystickButton(leftStick, MainControllerConstants.Thrustmaster.joyBUTTON_MIDDLE);
    leftMiddle.whenPressed(solenoidForward);

    JoystickButton rightMiddle = new JoystickButton(rightStick, MainControllerConstants.Thrustmaster.joyBUTTON_MIDDLE);
    rightMiddle.whenPressed(solenoidReverse);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return driveDistance;
  }
}
