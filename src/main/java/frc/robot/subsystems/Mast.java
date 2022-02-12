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



/** Add your docs here. */
public class Mast extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax mastMotor;
  private RelativeEncoder mastEncoder;
  final boolean isInverted = true;

  
  public Mast(){
    mastMotor = new CANSparkMax(Constants.ClimberConstants.ID_MAST, MotorType.kBrushless);

    mastEncoder = mastMotor.getEncoder();
   

    SmartDashboard.putNumber("Mast Encoder Position", mastEncoder.getPosition());
    

    SmartDashboard.putNumber("Mast Velocity", mastEncoder.getVelocity());
    

  }

  public void mastExtend() {
    mastMotor.restoreFactoryDefaults();
    mastMotor.set(Constants.ClimberConstants.MAST_EX_SPEED);

  }

  public void mastRetract(boolean isInverted) {
  
    mastMotor.setInverted(isInverted);
    mastMotor.set(Constants.ClimberConstants.MAST_RE_SPEED);
  
  }

  public void stop() {
    mastMotor.set(0);
  }



 
}
