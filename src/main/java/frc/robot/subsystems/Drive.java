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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private CANSparkMax leftFront, leftRear, rightFront, rightRear;
  private RelativeEncoder leftEncoder, rightEncoder;
  private SparkMaxPIDController leftPID, rightPID;
  private PIDController m_pidController;
  
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

    leftPID = leftFront.getPIDController();
    rightPID = rightFront.getPIDController();

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

    m_pidController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    m_pidController.setSetpoint(DriveConstants.DISTANCE);

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

  public void setRawSetPoint () {
    leftPID.setReference((10), ControlType.kPosition);
    rightPID.setReference((10), ControlType.kPosition);
  }

  public void setTurnSetPoint (double dist) {
    leftPID.setReference((dist * DriveConstants.IN_TO_REV_K), ControlType.kPosition);
    rightPID.setReference((-dist * DriveConstants.IN_TO_REV_K), ControlType.kPosition);
  }
  
  public void setOutputRange (double minOutput, double maxOutput) {
    leftPID.setOutputRange(minOutput, maxOutput);
    rightPID.setOutputRange(minOutput, maxOutput);
  }

  // SPARK MOTION CONTROL
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

  public void setkIz(double kIz) {
    leftPID.setIZone(kIz);
    rightPID.setIZone(kIz);
  }
  
  public void setkF(double kF) {
    leftPID.setFF(kF);
    rightPID.setFF(kF);
  }

  public double pidLOut() {
    return m_pidController.calculate(leftEncoder.getPosition());
  }

  public double pidROut() {
    return m_pidController.calculate(rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
