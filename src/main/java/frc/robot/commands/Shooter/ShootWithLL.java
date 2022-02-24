// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AngleAndDistLL;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootWithLL extends SequentialCommandGroup {
  /** Creates a new ShootWithLL. */
  public ShootWithLL(Drive drive, LoadingSpoon loadingSpoon, Shooter shooter, double botSpeed, double topSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AngleAndDistLL(drive).withTimeout(2),
      new SpoonAndShoot(loadingSpoon, shooter, botSpeed, topSpeed).withTimeout(5)
    );
  }
}
