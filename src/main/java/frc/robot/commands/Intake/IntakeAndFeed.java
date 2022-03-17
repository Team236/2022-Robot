// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndFeed extends ParallelCommandGroup {
  /** Creates a new IntakeAndFeed. */
  public IntakeAndFeed(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new NewIntakeForward(intake, IntakeConstants.FORWARD_SPEED),
      new SetFeedSpeeds(intake, IntakeConstants.FIRST_FEED_SPEED, IntakeConstants.SECOND_FEED_SPEED)
    );
  }
}
