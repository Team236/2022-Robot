// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Catapult extends SubsystemBase {

  public DoubleSolenoid solenoid;
  /** Creates a new PneumaticTest. */
  public Catapult() {
    // solenoid = new DoubleSolenoid(moduleType, 2, 2;)
  }

  public void forward() {
    solenoid.set(Value.kForward);
  }

  public void reverse() {
    solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 
  }
}
