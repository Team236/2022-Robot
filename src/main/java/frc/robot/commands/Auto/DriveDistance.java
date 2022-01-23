// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveWithPID;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistance extends SequentialCommandGroup {
  
  /** Creates a new DriveWithPID. */
  public DriveDistance(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // possible auto routine: shoot ball, back up, intake ball, limelight, shoot ball
    // or : back up, intake ball, limelight, shoot 2 balls

    super(sequence (new DriveWithPID(drive, 48, 3)), new WaitCommand(3), new DriveWithPID(drive, 24, 3).withTimeout(10));
  }
}
