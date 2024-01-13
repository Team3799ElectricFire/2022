/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

/**
 * This command sets the current position of the modules as the zero
 * degrees. All the modules should be facing forward when running 
 * this command. This command can be run while the robot is disabled.
 *
 * As a safety, so that the modules are not zeroed without cause, the 
 * command waits 10 seconds before the modules are zeroed. If another 
 * drive command is caled or this command is cancelled the modules' 
 * zeros will not be reset. Further, the robot MUST be disabled when 
 * this happens, or the modules will not have their zeros reset.
 */
public class Drive_ResetAllModulePositionsToZero extends CommandBase {
  private final DriveTrain _subsystem;
  Timer timer = new Timer();
  /**
   * Creates a new DriveResetAllModulePositionsToZero.
   */
  public Drive_ResetAllModulePositionsToZero(DriveTrain subsystem) {
    _subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Call the reset function to zero all of the modules 
    timer.reset();
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if(!interrupted && DriverStation.isDisabled()){
      _subsystem.zeroAllModulePosSensors();
    }
  }

  @Override
  public boolean isFinished() {
    return (timer.get() >= 10) || !DriverStation.isDisabled();
  }

  public boolean runsWhenDisabled(){
    return false;
  }
}