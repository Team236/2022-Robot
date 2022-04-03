// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
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

  private CANSparkMax intakeMotor, firstFeeder;
  private RelativeEncoder feedEncoder;
  private DoubleSolenoid intakeSolenoid;
  private boolean isIntkCounterUnplugged = false;
  private boolean isFeedCounterUnplugged = false;
  private ColorSensorV3 colorSensor;
  private Counter intakeEye, feederEye;
 
  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new CANSparkMax(MotorControllers.INTAKE, MotorType.kBrushless);
    intakeMotor.setInverted(true);

    firstFeeder = new CANSparkMax(MotorControllers.FIRST_FEEDER, MotorType.kBrushless);
    firstFeeder.setInverted(false);

    feedEncoder = firstFeeder.getEncoder();

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoids.INTAKE_SOL_FOR, Solenoids.INTAKE_SOL_REV);

    try {
      intakeEye = new Counter();
      intakeEye.setUpSource(IntakeConstants.DIO_INTAKE_EYE);
    } catch (Exception e) {
      isIntkCounterUnplugged = true;
    }

    try {
      feederEye = new Counter();
      feederEye.setUpSource(IntakeConstants.DIO_FEEDER_EYE);
    } catch (Exception e) {
      isFeedCounterUnplugged = true;
    }

    // I2C port on the NavX Gyro is known to be more reliable than the I2C port directly on the RoboRio
    I2C.Port onGyro = I2C.Port.kMXP;
    I2C.Port onRio = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(onGyro);

    intakeEye.reset();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setFirstFeedSpeed(double speed) {
    firstFeeder.set(speed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void stopFirstFeed() {
    firstFeeder.set(0);
  }

  public void extend() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retract() {
    intakeSolenoid.set(Value.kReverse);
  }

  public boolean isExtended() {
    if (intakeSolenoid.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public int getIntakeCount() {
    if (isIntkCounterUnplugged) {
      SmartDashboard.putBoolean("intake eye unplugged", isIntkCounterUnplugged);
      return 0;
    } else {
      return intakeEye.get();
    }
  }

  public int getFeederCount() {
    if (isFeedCounterUnplugged) {
      SmartDashboard.putBoolean("feed eye unplugged", isFeedCounterUnplugged);
      return 0;
    } 
    else {
      return feederEye.get();
    }
  }

  public void resetIntkCounter() {
    intakeEye.reset();
  }

  public void resetFeedCounter() {
    feederEye.reset();
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
    // large values mean object is close; small values mean object is far away
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

  public double getFeedVelocity() {
    return feedEncoder.getVelocity();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake sensor count", getIntakeCount());
    // SmartDashboard.putNumber("feeder sensor count", getFeederCount());
    // SmartDashboard.putNumber("color sense dist", colorSensor.getProximity());
    // SmartDashboard.putNumber("feed wheel speed", getFeedVelocity());
    // whatColor();
  }
}
