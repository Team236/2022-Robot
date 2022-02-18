// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorSensorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorControllers;
import frc.robot.Constants.Solenoids;;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private DoubleSolenoid intakeSolenoid;
  private boolean isCounterUnplugged = false;
  private ColorSensorV3 colorSensor;
  private Counter ballCounter;
 
  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new CANSparkMax(MotorControllers.INTAKE, MotorType.kBrushed);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoids.INTAKE_SOL_FOR, Solenoids.INTAKE_SOL_REV);

    try {
      ballCounter = new Counter();
      ballCounter.setUpSource(IntakeConstants.DIO_INTAKE_COUNTER);
      // this.ballCounter.setExternalDirectionMode();
    } catch (Exception e) {
      isCounterUnplugged = true;
    }

    I2C.Port onGyro = I2C.Port.kMXP;
    I2C.Port onRio = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(onGyro);

    ballCounter.reset();
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void extend() {
    intakeSolenoid.set(Value.kForward);
  }

  public boolean isExtended() {
    if (intakeSolenoid.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public void retract() {
    intakeSolenoid.set(Value.kReverse);
  }

  public int getBallCount() {
    if (isCounterUnplugged) {
      return 0;
    } else {
      return ballCounter.get();
    }
  }

  public void resetCounter() {
    ballCounter.reset();
  }

  public void getColor() {
    Color detectedColor = colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  }

  public double getRed() {
    Double redDouble = colorSensor.getColor().red;
    return redDouble;
  }

  public double getBlue() {
    Double blueDouble = colorSensor.getColor().blue;
    return blueDouble;
  }

  public boolean isRedGreater() {
    return (getRed() > getBlue());
  }

  public boolean isBlueGreater() {
    return (getBlue() > getRed());
  }

  public int getDistance() {
    SmartDashboard.putNumber("color sensor distance", colorSensor.getProximity());
    return colorSensor.getProximity();
  }

  public boolean isBallInSpoon() {
    return (getDistance() > ColorSensorConstants.DIST);
  }

  public void whatColor() {
    if (getDistance() < ColorSensorConstants.DIST) {
      SmartDashboard.putBoolean("blue ball in spoon", false);
      SmartDashboard.putBoolean("red ball in spoon", false);
    } else if ((getDistance() > ColorSensorConstants.DIST) && (isRedGreater())) {
      SmartDashboard.putBoolean("blue ball in spoon", false);
      SmartDashboard.putBoolean("red ball in spoon", true);
    } else if ((getDistance() > ColorSensorConstants.DIST) && (isBlueGreater())) {
      SmartDashboard.putBoolean("blue ball in spoon", true);
      SmartDashboard.putBoolean("red ball in spoon", false);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("optical sensor count", ballCounter.get());
    whatColor();
    SmartDashboard.putNumber("ball counter distance", ballCounter.getDistance());
  }
}
