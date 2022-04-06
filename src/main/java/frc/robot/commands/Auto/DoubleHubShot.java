// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Hood.HoodRetract;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.NewIntakeForward;
import frc.robot.commands.Shooter.FeedAndShoot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleHubShot extends SequentialCommandGroup {
  /** Creates a new DoubleHubShot. */
  public DoubleHubShot(Drive drive, Shooter shooter, Hood hood, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // hood at steeper angle
    addCommands(
      sequence(
        new IntakeExtend(intake).withTimeout(1),
        parallel(
          new NewIntakeForward(intake),
          new WPI_PID(drive, DriveConstants.TARMAC_TO_BALL_SHORT),
          new HoodRetract(hood)
        ).withTimeout(1.5),
        new WPI_PID(drive, -DriveConstants.HUB_TO_BALL).withTimeout(3),
        new FeedAndShoot(intake, shooter, hood, ShooterConstants.LOW_HUB_BOT, ShooterConstants.LOW_HUB_TOP)
      )
    );
  }
}
