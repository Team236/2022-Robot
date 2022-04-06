// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Arm extends SubsystemBase {
  
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armPID;
  private DigitalInput armReturnLimit, armExtendLimit;
  private boolean isAReturnUnplugged = false;
  private boolean isAExtendUnplugged = false;

  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax (frc.robot.Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(false);
    armMotor.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT_CLIMB);

    armPID = armMotor.getPIDController();

    armEncoder = armMotor.getEncoder();

    try {
      armExtendLimit = new DigitalInput(ClimberConstants.DIO_ARM_EXTEND);
    } catch (Exception e) {
      isAExtendUnplugged = true;
    }

    try {
      armReturnLimit = new DigitalInput(ClimberConstants.DIO_ARM_RETRACT);
    } catch (Exception e) {
      isAReturnUnplugged = true;
    }
  }

  public void armOut() {
    armMotor.set(ClimberConstants.ARM_EX_SPEED);
  }

  public void armIn() {
    armMotor.set(-ClimberConstants.ARM_RE_SPEED);
  }

  public void armStop() {
    armMotor.set(0);
  }

  public void resetArmEncoder() {
    armEncoder.setPosition(0);
  }

  public double getArmEncoder() {
    return armEncoder.getPosition();
  }

  public double getArmDistance() {
    return getArmEncoder() * ClimberConstants.armREV_TO_IN;
  }

  public void setArmSetPoint(double armDistance) {
    armPID.setReference((armDistance), ControlType.kPosition);
  }

  public void setArmkP(double kParm) {
    armPID.setP(kParm);
  }

  public void setArmkI(double kIarm) {
    armPID.setI(kIarm);
  }

  public void setArmkD(double kDarm) {
    armPID.setD(kDarm);
  }

  public void setArmkF(double kFarm) {
    armPID.setFF(kFarm);
  }

  public void setArmOutputRange() {
    armPID.setOutputRange(ClimberConstants.climberMIN_OUTPUT, ClimberConstants.climberMAX_OUTPUT);
  }

  public boolean isAReturnLimit() {
    if (isAReturnUnplugged) {
      return true;
    } else {
      return armReturnLimit.get();
    }
  }

  public boolean isAExtendLimit() {
    if (isAExtendUnplugged) {
      return true;
    } else {
      return armExtendLimit.get();
    }
  }

  public void setArmSpeed(double speed) {
    if (speed > 0) {
      if (isAReturnLimit()) {
        armStop();
      } else {
        armMotor.set(speed);
      }
    } else {
      if (isAExtendLimit()) {
        armStop();
        resetArmEncoder();
      } else {
        armMotor.set(speed);
      }
    }
  }

  public void setArmSetPointWlimit(double armDistance) {
    if (armExtendLimit.get() || armReturnLimit.get()) {
      armStop();
    } else {
      armPID.setReference((armDistance * ClimberConstants.armIN_TO_REV), ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("arm encoder", getArmEncoder());
    
    // SmartDashboard.putBoolean("arm extend limit", isAExtendLimit());
    // SmartDashboard.putBoolean("arm return limit", isAReturnLimit());
  }
}
