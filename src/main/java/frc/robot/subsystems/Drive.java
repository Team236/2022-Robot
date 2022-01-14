// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.ConstructorParameters;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private CANSparkMax leftFront, leftRear, rightFront, rightRear;
  private RelativeEncoder leftEncoder, rightEncoder;
  
  /** Creates a new Drive. */
  public Drive() {

    leftFront = new CANSparkMax(DriveConstants.ID_LEFT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(DriveConstants.ID_LEFT_REAR, MotorType.kBrushless);
    rightFront = new CANSparkMax(DriveConstants.ID_RIGHT_FRONT, MotorType.kBrushless);
    rightRear = new CANSparkMax(DriveConstants.ID_RIGHT_REAR, MotorType.kBrushless);

    leftFront.setInverted(false);
    rightFront.setInverted(true);

    leftRear.follow(leftFront, false);
    rightRear.follow(rightFront, false);

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

  }

  public void setLeftSpeed(double speed) {
    leftFront.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightFront.set(speed);
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
  
  // I am leaving out the code for the gyro ... is it necessasry?
  // why do the getEncoder functions multiply by revolutions to inches??
  public double getLeftEncoder() {
    return leftEncoder.getPosition() * DriveConstants.REV_TO_IN_K;
  }

  public double getRightEncoder() {
    return rightEncoder.getPosition() * DriveConstants.REV_TO_IN_K;
  }

  public double getRightDistance() {
    return getRightEncoder() * DriveConstants.REV_TO_IN_K;
  }

  public double getLeftDistance() {
    return getLeftEncoder() * DriveConstants.REV_TO_IN_K;
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

  // // PID
  // @Override
  // public void pidSet() {
  //   setRightSpeed(-speed);
  //   setLeftSpeed(speed);
  // }

  // // SPARK MOTION CONTROL
  // public void setkP(double kP) {
  //   leftPID.setP(kP);
  //   rightPID.setP(kP);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Drive", getLeftEncoder());
    SmartDashboard.putNumber("Right Drive", getRightEncoder());
  }
}
