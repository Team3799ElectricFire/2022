// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Intake_AdvanceCargo extends CommandBase {
  private final Intake _intake;
  private boolean lastReading;

  /** Creates a new Intake_AdvanceCargo. */
  public Intake_AdvanceCargo(Intake subsystem) {
    _intake = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _intake.IntakeToLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastReading = _intake.GetBreakBeamBack();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.IntakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Last reading didn't see cargo, this reading does, second cargo has arrived
    // Call this command with a timeout to make sure it ends even if there was only one cargo loaded
    return !lastReading && _intake.GetBreakBeamBack();
  }
}
