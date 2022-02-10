// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkMax bottomRoller, topRoller;
  private SparkMaxPIDController botPidController, topPidController;
  private RelativeEncoder botEncoder, topEncoder;

  /** Creates a new Shooter. */
  public Shooter() {

    bottomRoller = new CANSparkMax(1, MotorType.kBrushless);
    topRoller = new CANSparkMax(7, MotorType.kBrushless);

    // bottomRoller.restoreFactoryDefaults();
    // topRoller.restoreFactoryDefaults();

    botPidController = bottomRoller.getPIDController();
    topPidController = topRoller.getPIDController();

    botEncoder = bottomRoller.getEncoder();
    topEncoder = topRoller.getEncoder();

  }

  public void setBotSetPoint(double botSpeed) {
    botPidController.setReference(botSpeed, ControlType.kVelocity);
  }

  public void setTopSetPoint(double topSpeed) {
    topPidController.setReference(topSpeed, ControlType.kVelocity);
  }

  public void setP(double kPBot, double kPTop) {
    botPidController.setP(kPBot);
    topPidController.setP(kPTop);
  }

  public void setI(double kIBot, double kITop) {
    botPidController.setI(kIBot);
    topPidController.setI(kITop);
  }

  public void setD(double kDBot, double kDTop) {
    botPidController.setD(kDBot);
    topPidController.setD(kDTop);
  }

  public void setFF(double kFBot, double kFTop) {
    botPidController.setFF(kFBot);
    topPidController.setFF(kFTop);
  }

  public void setOutputRange() {
    topPidController.setOutputRange(0.0, 5000.0);
    botPidController.setOutputRange(0.0, 5000.0);
  }

  public void resetEncoders() {
    botEncoder.setPosition(0);
    topEncoder.setPosition(0);
  }

  public double getBotEncoder() {
    return botEncoder.getPosition();
  }

  public double getTopEncoder() {
    return topEncoder.getPosition();
  }

  public double getBotVelocity() {
    return botEncoder.getVelocity();
  }

  public double getTopVelocity() {
    return topEncoder.getVelocity();
  }

  public void setMotorSpeeds(double botSpeed, double topSpeed) {
    bottomRoller.set(botSpeed);
    topRoller.set(topSpeed);
  }

  public void setBotRawSpeed(double botSpeed) {
    if (botSpeed == 0) {
      bottomRoller.set(0);
    } else {
      if (botEncoder.getVelocity() > botSpeed) {
        bottomRoller.set(0);
      } else {
        bottomRoller.set(1);
      }
    }
  }

  public void setTopRawSpeed(double topSpeed) {
    if (topSpeed == 0) {
      topRoller.set(0);
    } else {
      if (topEncoder.getVelocity() > topSpeed) {
        topRoller.set(0);
      } else {
        topRoller.set(1);
      }
    }
  }

  public void stop() {
    setMotorSpeeds(0, 0);
  }

  public void restoreFactoryDefaults() {
      bottomRoller.restoreFactoryDefaults();
      topRoller.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("bot shoot velocity", getBotVelocity());
    SmartDashboard.putNumber("top shoot velocity", getTopVelocity());
  }
}
