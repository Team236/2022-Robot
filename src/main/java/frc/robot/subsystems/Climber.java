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

public class Climber extends SubsystemBase {

  private CANSparkMax armMotor, mastMotor;
  private RelativeEncoder armEncoder, mastEncoder;
  private SparkMaxPIDController armPID, mastPID;
  private DigitalInput mastReturnLimit, mastExtendLimit;
  private DigitalInput armReturnLimit, armExtendLimit;
  private boolean isMReturnUnplugged = false;
  private boolean isMExtendUnplugged = false;
  private boolean isAReturnUnplugged = false;
  private boolean isAExtendUnplugged = false;

  /** Creates a new Climber. */
  public Climber() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    mastMotor = new CANSparkMax(Constants.MotorControllers.ID_MAST, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    mastMotor.restoreFactoryDefaults();

    mastMotor.setInverted(false);
    armMotor.setInverted(false);

    armMotor.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT_CLIMB);
    mastMotor.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT_CLIMB);

    mastPID = mastMotor.getPIDController();
    armPID = armMotor.getPIDController();

    mastEncoder = mastMotor.getEncoder();
    armEncoder = armMotor.getEncoder();

    // mastReturnLimit = new DigitalInput(ClimberConstants.DIO_MAST_RETURN);
    // mastExtendLimit = new DigitalInput(ClimberConstants.DIO_MAST_EXTEND);

    // right now, both limits are wired as normally open
    // when they are open, the limit.get = true

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

  public void mastOut() {
    mastMotor.set(ClimberConstants.MAST_EX_SPEED);
  }

  public void armOut() {
    armMotor.set(ClimberConstants.ARM_EX_SPEED);
  }

  public void mastIn() {
    mastMotor.set(-ClimberConstants.MAST_RE_SPEED);
  }

  public void armIn() {
    armMotor.set(-ClimberConstants.ARM_RE_SPEED);
  }

  public void mastStop() {
    mastMotor.set(0);
  }

  public void armStop() {
    armMotor.set(0);
  }

  public void resetEncoders() {
    mastEncoder.setPosition(0);
    armEncoder.setPosition(0);
  }

  public void resetMastEncoder() {
    mastEncoder.setPosition(0);
  }

  public void resetArmEncoder() {
    armEncoder.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS 
  public double getMastEncoder() {
     return mastEncoder.getPosition();
  }

  public double getArmEncoder() {
    return armEncoder.getPosition();
  }

  public double getMastDistance() {
    return getMastEncoder() * ClimberConstants.mastREV_TO_IN;
  } 

  public double getArmDistance() {
    return getArmEncoder() * ClimberConstants.armREV_TO_IN;
  }

  //different distances for each instance of PID
  //armDistance is in INCHES not revs
  public void setArmSetPoint(double armDistance) {
   armPID.setReference((armDistance * ClimberConstants.armIN_TO_REV), ControlType.kPosition);
  }
 
  public void setMastSetPoint(double mastDistance) {
    // mastPID.setReference((mastDistance * ClimberConstants.mastIN_TO_REV), ControlType.kPosition); 
    mastPID.setReference(mastDistance, ControlType.kPosition);
  }

  public void setMastkP(double kPmast) {
    mastPID.setP(kPmast);
  }

  public void setArmkP(double kParm) {
    armPID.setP(kParm);
  }

  public void setMastkI(double kImast) {
    mastPID.setI(kImast);
  }

  public void setArmkI(double kIarm) {
    armPID.setI(kIarm);
  }

  public void setMastkD(double kDmast) {
    mastPID.setD(kDmast);
  }

  public void setArmkD(double kDarm) {
    armPID.setD(kDarm);
  }

  public void setMastkF(double kFmast) {
    mastPID.setFF(kFmast);
  }

  public void setArmkF(double kFarm) {
    armPID.setFF(kFarm);
  }

  public void setArmOutputRange() {
    armPID.setOutputRange(ClimberConstants.climberMIN_OUTPUT, ClimberConstants.climberMAX_OUTPUT);
  }

  public void setMastOutputRange() {
    mastPID.setOutputRange(ClimberConstants.climberMIN_OUTPUT, ClimberConstants.climberMAX_OUTPUT);
  }

  public boolean isMReturnLimit() {
    if (isMReturnUnplugged) {
      return true;
    } else {
      return mastReturnLimit.get();
    }
  }
  
  public boolean isMExtendLimit() {
    if (isMExtendUnplugged) {
      return true;
    } else {
      return mastExtendLimit.get();
    }
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

  public void setMastSpeed(double speed) {
    if (speed > 0) {
      if (mastExtendLimit.get()) {
        // mast going up and top limit is tripped, stop
        mastStop();
      } else {
        // mast going up but top limit is not tripped, go at commanded speed
        mastMotor.set(speed);
      }
    } else {
      if (mastReturnLimit.get()) {
        // mast going down and bottom limit is tripped, stop and zero encoder
        mastStop();
        resetMastEncoder();
      } else {
        // mast going down but bottom limit is not tripped, go at commanded speed
        mastMotor.set(speed);
      }
    }
  }

  public void setArmSpeed(double speed) {
    if (speed > 0) {
      if (armExtendLimit.get()) {
        armStop();
      } else {
        armMotor.set(speed);
      }
    } else {
      if (armReturnLimit.get()) {
        armStop();
        resetArmEncoder();
      } else {
        armMotor.set(speed);
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
    SmartDashboard.putNumber("mast encoder", mastEncoder.getPosition());
    SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());

    SmartDashboard.putBoolean("mast extend limit", isMExtendLimit());
    SmartDashboard.putBoolean("mast return limit", isMReturnLimit());
    
    SmartDashboard.putBoolean("arm extend limit", isAExtendLimit());
    SmartDashboard.putBoolean("arm return limit", isAReturnLimit());
  }

}
