// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Shooter.FeedAndShoot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootMoveShoot extends SequentialCommandGroup {
  /** Creates a new MoveAndShootTwice. */
  public ShootMoveShoot(Drive drive, Shooter shooter, Hood hood, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new WPI_PID(drive, DriveConstants.TARMAC_TO_LINE).withTimeout(2),
      new FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP),
      new IntakeExtend(intake).withTimeout(1),
      parallel(
        new AutoIntake(intake),
        new WPI_PID(drive, DriveConstants.BALL_TO_LINE)
      ).withTimeout(4),
      new WaitCommand(0.5),
      new WPI_PID(drive, -DriveConstants.BALL_TO_LINE).withTimeout(2),
      new FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP).withTimeout(3)
    );
  }
}
