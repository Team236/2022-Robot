// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Drive.WPI_Turn_PID;
import frc.robot.commands.Hood.HoodRetract;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Shooter.FeedAndShoot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TriplePosition1 extends SequentialCommandGroup {
  /** Creates a new DoubleTarmac3. */
  public TriplePosition1(Intake intake, Drive drive, Hood hood, LoadingSpoon loadingSpoon, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeExtend(intake).withTimeout(0.5),
      parallel(
        new AutoIntake(intake),
        new WPI_PID(drive, DriveConstants.TARMAC_TO_BALL_SHORT),
        new HoodRetract(hood)
      ).withTimeout(1.5),
      new FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP).withTimeout(2.2),
      parallel(
        new WPI_Turn_PID(drive, 98),
        new IntakeExtend(intake)
      ).withTimeout(1),
      parallel(
        new AutoIntake(intake),
        sequence(
          new WPI_PID(drive, 95).withTimeout(1.5),
          new WPI_Turn_PID(drive, -67).withTimeout(1)
        )
      ).withTimeout(2.5),
      new FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP)
    );
  }
}
