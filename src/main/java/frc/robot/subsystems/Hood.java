// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

  DoubleSolenoid hoodDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HoodConstants.HOOD_EXTEND,  Constants.HoodConstants.HOOD_RETRACT);

  
  /** Creates a new Hood. */
  public Hood() {

  }
  public void hoodExtend() {
    hoodDoubleSolenoid.set(Value.kForward);
  }
  public void hoodRetract() {
    hoodDoubleSolenoid.set(Value.kReverse);
  }
  public void hoodOff() {
    hoodDoubleSolenoid.set(Value.kOff);
  }
  public boolean hoodIsExtended() {
    if(hoodDoubleSolenoid.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
