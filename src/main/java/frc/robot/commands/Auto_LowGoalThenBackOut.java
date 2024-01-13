// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_LowGoalThenBackOut extends SequentialCommandGroup {
  /** Creates a new Auto_LowGoalThenBackOut. */
  public Auto_LowGoalThenBackOut(DriveTrain driveTrain, Intake intake, Launcher launcher) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(intake::JawClose,intake),
      new Auto_DriveSwerve(driveTrain, 0.5),
      new InstantCommand(launcher::SetLowTarget,launcher),
      new Launcher_ReadyCannon(launcher),
      new Intake_CargoToLauncher(intake).withTimeout(2.0),
      new Auto_DriveSwerve(driveTrain, 1.5)
    );
  }
}
