// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Spoon.SpoonExtend;
import frc.robot.commands.Spoon.SpoonRetract;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpoonAndShoot extends ParallelCommandGroup {
  /** Creates a new SpoonAndShoot. */
  public SpoonAndShoot(LoadingSpoon loadingSpoon, Shooter shooter, double botSpeed, double topSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // runs shooter in parallel with wait/spoonExtend/spoonRetract sequence
    addCommands(
      new Shoot(shooter, botSpeed, topSpeed), 
      sequence(
        new WaitCommand(1), 
        new SpoonExtend(loadingSpoon).withTimeout(0.5), 
        new SpoonRetract(loadingSpoon).withTimeout(0.5)
          )
        );
  }
}
