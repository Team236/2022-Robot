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

  // private SparkMaxLimitSwitch bottomLimit;
  private DigitalInput topLimit, bottomLimit;
  private boolean isLimitUnplugged = false;

  /** Creates a new Climber. */
  public Climber() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    mastMotor = new CANSparkMax(Constants.MotorControllers.ID_MAST, MotorType.kBrushless);

    mastMotor.setInverted(false);
    armMotor.setInverted(false);

    mastPID = mastMotor.getPIDController();
    armPID = armMotor.getPIDController();

    mastEncoder = mastMotor.getEncoder();
    armEncoder = armMotor.getEncoder();

    topLimit = new DigitalInput(ClimberConstants.DIO_TOP_LIM);
    // try {
    //   topLimit = new DigitalInput(ClimberConstants.DIO_TOP_LIM);
    // } catch (Exception e) {
    //   isLimitUnplugged = true;
    // }
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
    mastMotor.set(0);
  }

  public void resetEncoders() {
    mastEncoder.setPosition(0);
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
    mastPID.setReference((mastDistance * ClimberConstants.armIN_TO_REV), ControlType.kPosition); 
  } 

  public void setMastSetPointWlimit(double mastDistance) {
    if (topLimit.get()) {
      mastPID.setReference((mastDistance * ClimberConstants.armIN_TO_REV), ControlType.kPosition);
      // mastIn();
    } else {
      mastStop();
    }
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

  // public boolean isTopLimit() {
  //   if (isLimitUnplugged) {
  //     return false;
  //   } else {
  //     return !topLimit.get(); //maybe should be !topLimit.get()
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("top limit", topLimit.get());
  }

}
