// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorControllers;
import frc.robot.Constants.DriveConstants;;

public class Drive extends SubsystemBase {

  public CANSparkMax leftFront, leftRear, rightFront, rightRear;
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkMaxPIDController leftPID, rightPID;
  
  /** Creates a new Drive. */
  public Drive() {

    leftFront = new CANSparkMax(MotorControllers.ID_LEFT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(MotorControllers.ID_LEFT_REAR, MotorType.kBrushless);
    rightFront = new CANSparkMax(MotorControllers.ID_RIGHT_FRONT, MotorType.kBrushless);
    rightRear = new CANSparkMax(MotorControllers.ID_RIGHT_REAR, MotorType.kBrushless);

    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    
    leftFront.setInverted(false);
    rightFront.setInverted(true);
    
    leftRear.follow(leftFront, false);
    rightRear.follow(rightFront, false);

    leftFront.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
    leftRear.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
    rightFront.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
    rightRear.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);

    leftPID = leftFront.getPIDController();
    rightPID = rightFront.getPIDController();

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

  }

  public void setLeftSpeed(double speed) {
    leftFront.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightFront.set(speed);
  }

  public void setBothSpeeds(double speed) {
    leftFront.set(speed);
    rightFront.set(speed);
  }

  public void setTurnSpeeds(double speed) {
    leftFront.set(speed);
    rightFront.set(-speed);
  }

  public void setLeftSpeedWithDeadzone(double speed) {
    double leftSpeed = speed;
    if (leftSpeed < DriveConstants.LEFT_DEADZONE && leftSpeed > -DriveConstants.LEFT_DEADZONE) {
      leftSpeed = 0;
    }
    setLeftSpeed(leftSpeed);
  }

  public void setRightSpeedWithDeadzone(double speed) {
    double rightSpeed = speed;
    if (rightSpeed < DriveConstants.RIGHT_DEADZONE && rightSpeed > -DriveConstants.RIGHT_DEADZONE) {
      rightSpeed = 0;
    }
    setRightSpeed(rightSpeed);
  }

  public void stop() {
    setLeftSpeed(0);
    setRightSpeed(0);
  }

  public void closedRampRate() {
    leftFront.setClosedLoopRampRate(0.08);
    rightFront.setClosedLoopRampRate(0.08);
  }

  public void openRampRate() {
    leftFront.setOpenLoopRampRate(0.08);
    rightFront.setOpenLoopRampRate(0.08);
  }
  
  // getEncoder methods return encoder position in ROTATIONS of motor
  public double getLeftEncoder() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }

  // getDistance methods return encoder position in INCHES
  public double getRightDistance() {
    return getRightEncoder() * DriveConstants.REV_TO_IN_K;
  }

  public double getLeftDistance() {
    return getLeftEncoder() * DriveConstants.REV_TO_IN_K;
  }

  public double getAvgDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  // PID
  // sets PID setpoint to a value of revolutions given a distance in inches
  public void setSetPoint (double dist) {
    leftPID.setReference((dist * DriveConstants.IN_TO_REV_K), ControlType.kPosition);
    rightPID.setReference((dist * DriveConstants.IN_TO_REV_K), ControlType.kPosition);
  }

  public void setTurnSetPoint (double dist) {
    leftPID.setReference((dist * DriveConstants.IN_TO_REV_K), ControlType.kPosition);
    rightPID.setReference((-dist * DriveConstants.IN_TO_REV_K), ControlType.kPosition);
  }
  
  public void setOutputRange (double minOutput, double maxOutput) {
    leftPID.setOutputRange(minOutput, maxOutput);
    rightPID.setOutputRange(minOutput, maxOutput);
  }

  // SparkMax PID Control   *** not reliable - better to use WPILib PID control
  public void setkP(double kP) {
    leftPID.setP(kP);
    rightPID.setP(kP);
  }
  
  public void setkI(double kI) {
    leftPID.setI(kI);
    rightPID.setI(kI);
  }  
  
  public void setkD(double kD) {
    leftPID.setD(kD);
    rightPID.setD(kD);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
