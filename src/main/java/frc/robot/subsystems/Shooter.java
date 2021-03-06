// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorControllers;

public class Shooter extends SubsystemBase {

  private CANSparkMax bottomRoller, topRoller;
  private SparkMaxPIDController botPidController, topPidController;
  private RelativeEncoder botEncoder, topEncoder;

  /** Creates a new Shooter. */
  public Shooter() {

    bottomRoller = new CANSparkMax(MotorControllers.BOTTOM_SHOOTER, MotorType.kBrushless);
    topRoller = new CANSparkMax(MotorControllers.TOP_SHOOTER, MotorType.kBrushless);

    topRoller.setInverted(true);

    // bottomRoller.restoreFactoryDefaults();
    // topRoller.restoreFactoryDefaults();

    botPidController = bottomRoller.getPIDController();
    topPidController = topRoller.getPIDController();

    botEncoder = bottomRoller.getEncoder();
    topEncoder = topRoller.getEncoder();
  }

  // sets RPM velocity set point
  public void setBotSetPoint(double botSpeed) {
    botPidController.setReference(botSpeed, ControlType.kVelocity);
  }

  public void setTopSetPoint(double topSpeed) {
    topPidController.setReference(topSpeed, ControlType.kVelocity);
  }

  // sets PIDF values for each motor
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

  // sets max and min rpm setpoints of motors
  public void setOutputRange() {
    topPidController.setOutputRange(0, 6000);
    botPidController.setOutputRange(0, 6000);
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
