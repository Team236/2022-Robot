// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.DriveWithPID;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.IntakeForward;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Shooter.SpoonAndShoot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootMoveShoot extends SequentialCommandGroup {
  /** Creates a new MoveAndShootTwice. */
  public ShootMoveShoot(Drive drive, Shooter shooter, Hood hood, LoadingSpoon loadingSpoon, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new WPI_PID(drive, DriveConstants.TARMAC_TO_LINE).withTimeout(2),
      new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.HIGH_HUB_BOT, ShooterConstants.HIGH_HUB_TOP).withTimeout(3),
      new WaitCommand(1),
      new IntakeExtend(intake).withTimeout(1),
      parallel(
        new SetIntakeSpeed(intake, IntakeConstants.FORWARD_SPEED),
        new WPI_PID(drive, DriveConstants.BALL_TO_LINE)
      ).withTimeout(4),
      new WaitCommand(1),
      new WPI_PID(drive, -DriveConstants.BALL_TO_LINE).withTimeout(2),
      new SpoonAndShoot(loadingSpoon, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP).withTimeout(6)
    );
  }
}
