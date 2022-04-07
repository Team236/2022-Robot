// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Drive.WPI_Turn_PID;
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
public class QuadruplePosition1 extends SequentialCommandGroup {
  /** Creates a new QuadruplePosition1. */
  public QuadruplePosition1(Drive drive, Intake intake, Hood hood, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      parallel(
        sequence(      
          new IntakeExtend(intake).withTimeout(0.5),
          new NewIntakeForward(intake)
        ),
        new WPI_PID(drive, DriveConstants.TARMAC_TO_BALL_SHORT),
        new HoodRetract(hood)
      ).withTimeout(1.5),
      new FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP).withTimeout(1.8),
      new WPI_Turn_PID(drive, 103).withTimeout(1),
      parallel(
        sequence(
          new IntakeExtend(intake).withTimeout(0.5),
          new NewIntakeForward(intake)
        ),
        sequence(
          new WPI_PID(drive, 100).withTimeout(1.5),
          new WPI_PID(drive, 70).withTimeout(1.2),
          new WPI_Turn_PID(drive, -50).withTimeout(1),
          new WPI_PID(drive, 100).withTimeout(2)
        )
      ).withTimeout(6),
      parallel(
        new WPI_PID(drive, -100),
        sequence(
          new WaitCommand(1),
          new FeedAndShoot(intake, shooter, hood, ShooterConstants.LAUNCH_PAD_BOT, ShooterConstants.LAUNCH_PAD_TOP)
        )
      )
    );
  }
}
