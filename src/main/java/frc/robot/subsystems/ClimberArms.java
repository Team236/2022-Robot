// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberArms extends SubsystemBase {
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  final boolean isInverted = true; 

  public ClimberArms() {
    
  armMotor = new CANSparkMax(Constants.ClimberConstants.ID_ARM, MotorType.kBrushless);

  armEncoder = armMotor.getEncoder();
  SmartDashboard.putNumber("Arm Encoder Position", armEncoder.getPosition());
  
  SmartDashboard.putNumber("Arm Velocity", armEncoder.getVelocity());
  
  }

  public void armExtend() {
    armMotor.restoreFactoryDefaults();
    armMotor.set(Constants.ClimberConstants.ARM_EX_SPEED);
  }
 
  public void armRetract(boolean isInverted) {
    armMotor.setInverted(isInverted);
    armMotor.set(Constants.ClimberConstants.ARM_RE_SPEED);
  }

  public void armStop() {
    armMotor.set(0);
  }
}
