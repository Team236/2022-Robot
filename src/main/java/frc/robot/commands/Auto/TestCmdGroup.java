// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveWithPID;
import frc.robot.commands.Drive.TurnWithPID;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestCmdGroup extends SequentialCommandGroup {
  
  /** Creates a new DriveWithPID. */
  public TestCmdGroup(Drive drive) {

    addCommands(
      new WPI_PID(drive, 48).withTimeout(3),
      new WaitCommand(2),
      new WPI_PID(drive, -20).withTimeout(2)
    );
  }
}
