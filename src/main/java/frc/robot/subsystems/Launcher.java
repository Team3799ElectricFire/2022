// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {

  private boolean isTargetingHighGoal = false;
  private final TalonFX leftLauncherMotor, rightLauncherMotor;

  public Launcher() {
    leftLauncherMotor = new TalonFX(Constants.LeftLauncherMotorCANID);
    rightLauncherMotor = new TalonFX(Constants.RightLauncherMotorCANID);
    
    leftLauncherMotor.configFactoryDefault();
    rightLauncherMotor.configFactoryDefault();

    leftLauncherMotor.setInverted(TalonFXInvertType.Clockwise);
    rightLauncherMotor.setInverted(TalonFXInvertType.CounterClockwise);

    leftLauncherMotor.setNeutralMode(NeutralMode.Coast);
    rightLauncherMotor.setNeutralMode(NeutralMode.Coast);

    leftLauncherMotor.follow(rightLauncherMotor);

    rightLauncherMotor.configOpenloopRamp(0.25);
    rightLauncherMotor.configClosedloopRamp(0.25);
    rightLauncherMotor.config_kP(0, 0.1);
    rightLauncherMotor.config_kI(0, 0.00001);
    rightLauncherMotor.config_kD(0, 10);
    rightLauncherMotor.config_kF(0, 1023.0/20660.0);
  }

  @Override
  public void periodic() {
    //UpdateDashboard();
  }

  public void CannonLaunch(){
    rightLauncherMotor.set(ControlMode.Velocity, isTargetingHighGoal ? Constants.LauncherHighSpeed : Constants.LauncherLowSpeed);
  }
  public void CannonStop() {
    rightLauncherMotor.set(ControlMode.Velocity, 0.0);
  }
  public void CannonEject() {
    rightLauncherMotor.set(ControlMode.Velocity, Constants.LauncherEjectSpeed);
  }
  public boolean CannonAtSpeed() {
    return rightLauncherMotor.getClosedLoopError() < 100;
  }

  public void ToggleHighLowTarget() {
    if (isTargetingHighGoal) {
      isTargetingHighGoal = false;
    } else {
      isTargetingHighGoal = true;
    }

    // You can also simplify the above, with either of these below -Dave
    //isTargetingHighGoal = isTargetingHighGoal ^ true;
    //isTargetingHighGoal ^= true;
  }
  public void SetHighTarget() {
    isTargetingHighGoal = true;
  }
  public void SetLowTarget() {
    isTargetingHighGoal = false;
  }

  // Smart Dashboard puts
  public void UpdateDashboard() {
    //SmartDashboard.putNumber("Cannon Target Speed", rightLauncherMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Cannon Speed", rightLauncherMotor.getSelectedSensorVelocity());
  }
}
