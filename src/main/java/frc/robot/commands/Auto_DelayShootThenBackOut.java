// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_DelayShootThenBackOut extends SequentialCommandGroup {
  /** Creates a new Auto_DelayThenBackOut. */
  public Auto_DelayShootThenBackOut(DriveTrain driveTrain, Intake intake, Launcher launcher, double delayTimeSeconds, double driveTimeSeconds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(delayTimeSeconds),

      new Auto_DriveSwerve(driveTrain, driveTimeSeconds*0.4),
      new InstantCommand(launcher::SetHighTarget,launcher),
      new Launcher_ReadyCannon(launcher),
      new Intake_CargoToLauncher(intake).withTimeout(1.5),
      new InstantCommand(launcher::CannonStop,launcher),
      new InstantCommand(intake::IntakeStop,intake),

      //new InstantCommand(intake::JawClose,intake),
      new Auto_DriveSwerve(driveTrain, driveTimeSeconds*0.6)
    );
  }
}
