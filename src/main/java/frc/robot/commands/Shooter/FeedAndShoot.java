// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Intake.ResetFeedCount;
import frc.robot.commands.Intake.SetFeedSpeed;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedAndShoot extends ParallelCommandGroup {
  /** Creates a new FeedAndShoot. */
  public FeedAndShoot(Intake intake, Shooter shooter, Hood hood, double botSpeed, double topSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Shoot(shooter, botSpeed, topSpeed),
      sequence(
        new WaitCommand(1),
        new SetFeedSpeed(intake, IntakeConstants.FIRST_FEED_SPEED)
      )
    );
  }
}
