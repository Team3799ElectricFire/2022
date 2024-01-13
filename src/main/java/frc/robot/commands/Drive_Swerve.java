// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

// Field oriented swerve drive based off controller's thumbsticks
public class Drive_Swerve extends CommandBase {
  private final DriveTrain _subsystem;
  private final DoubleSupplier _forwardSupplier, _strafeSupplier, _rotationSupplier;

  // Values for 340's counter-drag steering PID, concept taken from their GitHub page:
  // https://github.com/Greater-Rochester-Robotics/RapidReact2022-340/tree/6e6efe776226b61753306ac2b50287139e4e9bff/src/main/java/frc/robot
  //private double currentAngle = 0.0;
  //private boolean wasDriverControl;
  

  /** Creates a new DriveTrain_Swerve. */
  public Drive_Swerve(DriveTrain subsystem, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
    _subsystem = subsystem;
    _forwardSupplier = forwardSupplier;
    _strafeSupplier = strafeSupplier;
    _rotationSupplier = rotationSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    //currentAngle = _subsystem.GetGyroRad();
    //wasDriverControl = false;
  }

  @Override
  public void execute() {
    double forward, strafe, rotation;
    forward = -1 * _forwardSupplier.getAsDouble(); // Multiply by -1 b/c forward on the thumbsticks is negative (for some reason...)
    strafe = -1 * _strafeSupplier.getAsDouble(); // Multiply by -1 to make left positive
    rotation = -1 *  _rotationSupplier.getAsDouble(); // Multiply by -1 to make left/counterclockwise positive

    // Check if input is larger than deadband (otherwise stop)
    if ( (Math.sqrt(forward*forward + strafe*strafe) > Constants.ThumbstickDeadBand) || (Math.abs(rotation) > Constants.ThumbstickDeadBand) ) {

      if (_subsystem.GetFieldRelative()) {
        _subsystem.DriveSwerveFieldRelative(forward, strafe, rotation);
      } else {
        _subsystem.DriveSwerve(forward, strafe, rotation);
      }
      
    } else {
      _subsystem.Stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _subsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
