// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3.MainControl;

import com.revrobotics.ColorSensorV3.MainControl;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MainControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MainControllerConstants;
import frc.robot.commands.DashboardPID;
import frc.robot.commands.ElevatorExtend; 
import frc.robot.commands.ElevatorRetract;
import frc.robot.commands.DriveWithPID;
import frc.robot.commands.ElevatorExtend; 
import frc.robot.commands.ElevatorRetract;
import frc.robot.subsystems.Trigger;
import frc.robot.commands.SolenoidReverse;
import frc.robot.subsystems.Drive;
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * This class is where the bulk of the robot should be declared. Since
 * @param <DriveDistance>
 * @param <DriveDistance>
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
  private final Trigger trigger = new Trigger();
 * @param <DriveDistance>
 * @param <DriveDistance>
  Joystick controller = new Joystick(MainControllerConstants.USB_CONTROLLER);
//  Joystick leftStick = new Joystick(MainControllerConstants.USB_LEFT_STICK);
 // Joystick rightStick = new Joystick(MainControllerConstants.USB_RIGHT_STICK);

  // **SUBSYSTEMS**
  private final Drive drive = new Drive();
  private final Trigger trigger = new Trigger();
 // private final DriveDistance driveDistance = new DriveDistance(drive);
  // **JOYSTICKS**
 // private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
//  Joystick leftStick = new Joystick(MainControllerConstants.USB_LEFT_STICK);
 // Joystick rightStick = new Joystick(MainControllerConstants.USB_RIGHT_STICK);

//  private final SolenoidForward solenoidForward = new SolenoidForward();
 // private final SolenoidReverse solenoidReverse = new SolenoidReverse();
  //Trigger/Elevator
  private final ElevatorExtend elevatorExtend = new ElevatorExtend(trigger);
  private final ElevatorRetract elevatorRetract = new ElevatorRetract(trigger);
  // AUTO
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  // DRIVE
 // private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
    //drive.setDefaultCommand(driveWithJoysticks);
  private final DashboardPID dashboardPID = new DashboardPID(drive, DriveConstants.DISTANCE, DriveConstants.MARGIN);
  // PNEUMATICS
//  private final SolenoidForward solenoidForward = new SolenoidForward();
 // private final SolenoidReverse solenoidReverse = new SolenoidReverse();
  //Trigger/Elevator
  private final ElevatorExtend elevatorExtend = new ElevatorExtend(trigger);
  private final ElevatorRetract elevatorRetract = new ElevatorRetract(trigger);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

   /*JoystickButton leftTrigger = new JoystickButton(leftStick, MainControllerConstants.Thrustmaster.joyTRIGGER);

    // Configure the button bindings
    configureButtonBindings();
  }

   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   */
  private void configureButtonBindings() {

   /*JoystickButton leftTrigger = new JoystickButton(leftStick, MainControllerConstants.Thrustmaster.joyTRIGGER);
    leftTrigger.whenPressed(driveWithPID);

    JoystickButton rightTrigger = new JoystickButton(rightStick, MainControllerConstants.Thrustmaster.joyTRIGGER);
 // public Command getAutonomousCommand() {

    //return driveDistance;
 // }

   /* JoystickButton rightMiddle = new JoystickButton(rightStick, MainControllerConstants.Thrustmaster.joyBUTTON_MIDDLE);
    rightMiddle.whenPressed();
   
   JoystickButton rightRight = new JoystickButton(rightStick, MainControllerConstants.Thrustmaster.joyBUTTON_RIGHT);
   rightRight.whenPressed(solenoidReverse); */

    JoystickButton rb = new JoystickButton(controller, MainControllerConstants.LogitechF310.contRB);
    rb.whenPressed(elevatorRetract);
    
    JoystickButton lb = new JoystickButton(controller, MainControllerConstants.LogitechF310.contLB);
    lb.whenPressed(elevatorExtend);


    
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 // public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return driveDistance;
 // }
}
