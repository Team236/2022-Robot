// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Spoon;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LoadingSpoon;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpoonCmdGroup extends SequentialCommandGroup {
  /** Creates a new ExtendWaitRetract. */
  public SpoonCmdGroup(LoadingSpoon loadingSpoon) {
    // Add your commands in the addCommands() call

    addCommands(
      new SpoonExtend(loadingSpoon).withTimeout(0.5),
      new SpoonRetract(loadingSpoon).withTimeout(0.5)
    );
  }
}
