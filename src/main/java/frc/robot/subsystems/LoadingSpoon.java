// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Solenoids;

public class LoadingSpoon extends SubsystemBase {

  
  DoubleSolenoid spoonDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Solenoids.SPOON_SOL_FOR,  Solenoids.SPOON_SOL_REV);

  
  public LoadingSpoon() {
  }

  public void forward() {
    spoonDoubleSolenoid.set(Value.kForward);
  }

  public void reverse() {
    spoonDoubleSolenoid.set(Value.kReverse);
  }

  public Value getPosition() {
    return spoonDoubleSolenoid.get();
  }

  public void toggle() {
    spoonDoubleSolenoid.toggle();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
