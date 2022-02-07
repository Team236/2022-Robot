// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trigger extends SubsystemBase {

  
  DoubleSolenoid elevatorDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.TriggerConstants.TRIGGER_EXTEND,  Constants.TriggerConstants.TRIGGER_RETRACT);

  
  public Trigger() {
  }

  public void forward() {
    elevatorDoubleSolenoid.set(Value.kForward);
  }

  public void reverse() {
    elevatorDoubleSolenoid.set(Value.kReverse);
  }

  public void off() {
    elevatorDoubleSolenoid.set(Value.kOff);
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
