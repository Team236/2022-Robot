// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mast extends SubsystemBase {

  private CANSparkMax armMotor, mastMotor;
  private RelativeEncoder armEncoder, mastEncoder;
  private SparkMaxPIDController armPID, mastPID;
  private DigitalInput mastReturnLimit, mastExtendLimit;
  private boolean isMReturnUnplugged = false;
  private boolean isMExtendUnplugged = false;

  /** Creates a new Climber. */
  public Mast() {

    mastMotor = new CANSparkMax(Constants.MotorControllers.ID_MAST, MotorType.kBrushless);
    mastMotor.restoreFactoryDefaults();
    mastMotor.setInverted(false);
    mastMotor.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT_CLIMB);

    mastPID = mastMotor.getPIDController();

    mastEncoder = mastMotor.getEncoder();

    try {
      mastReturnLimit = new DigitalInput(ClimberConstants.DIO_MAST_RETURN);
    } catch (Exception e) {
      isMReturnUnplugged = true;
    }

    try {
      mastExtendLimit = new DigitalInput(ClimberConstants.DIO_MAST_EXTEND);
    } catch (Exception e) {
      isMExtendUnplugged = true;
    }

  }

  public void mastOut() {
    mastMotor.set(ClimberConstants.MAST_EX_SPEED);
  }

  public void mastIn() {
    mastMotor.set(-ClimberConstants.MAST_RE_SPEED);
  }

  public void mastStop() {
    mastMotor.set(0);
  }

  public void resetMastEncoder() {
    mastEncoder.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS 
  public double getMastEncoder() {
     return mastEncoder.getPosition();
  }

  public double getMastDistance() {
    return getMastEncoder() * ClimberConstants.mastREV_TO_IN;
  } 
 
  public void setMastSetPoint(double mastDistance) {
    // mastPID.setReference((mastDistance * ClimberConstants.mastIN_TO_REV), ControlType.kPosition); 
    mastPID.setReference(mastDistance, ControlType.kPosition);
  }

  public void setMastkP(double kPmast) {
    mastPID.setP(kPmast);
  }

  public void setMastkI(double kImast) {
    mastPID.setI(kImast);
  }

  public void setMastkD(double kDmast) {
    mastPID.setD(kDmast);
  }

  public void setMastkF(double kFmast) {
    mastPID.setFF(kFmast);
  }

  public void setMastOutputRange() {
    mastPID.setOutputRange(ClimberConstants.climberMIN_OUTPUT, ClimberConstants.climberMAX_OUTPUT);
  }

  public boolean isMReturnLimit() {
    if (isMReturnUnplugged) {
      return true;
    } else {
      return !mastReturnLimit.get();
    }
  }
  
  public boolean isMExtendLimit() {
    if (isMExtendUnplugged) {
      return true;
    } else {
      return !mastExtendLimit.get();
    }
  }

  public void setMastSpeed(double speed) {
    if (speed > 0) {
      if (isMExtendLimit()) {
        // mast going up and top limit is tripped, stop
        mastStop();
      } else {
        // mast going up but top limit is not tripped, go at commanded speed
        mastMotor.set(speed);
      }
    } else {
      if (isMReturnLimit()) {
        // mast going down and bottom limit is tripped, stop and zero encoder
        mastStop();
        resetMastEncoder();
      } else {
        // mast going down but bottom limit is not tripped, go at commanded speed
        mastMotor.set(speed);
      }
    }
  }

  public void setMastSetPointWlimit(double mastDistance) {
    if (mastExtendLimit.get() || mastReturnLimit.get()) {
      mastStop();
    } else {
      mastPID.setReference((mastDistance * ClimberConstants.mastIN_TO_REV), ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("mast encoder", getMastEncoder());

    SmartDashboard.putBoolean("mast extend limit", isMExtendLimit());
    SmartDashboard.putBoolean("mast return limit", isMReturnLimit());
    
  }

}
