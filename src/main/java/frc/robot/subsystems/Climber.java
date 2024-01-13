// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final DoubleSolenoid armsSolenoid, brakesSolenoid;
  private final TalonFX RightClimbMotor, LeftClimbMotor;

  /** Creates a new Climber. */
  public Climber() {
    LeftClimbMotor = new TalonFX(Constants.LeftClimbMotorCANID);
    RightClimbMotor = new TalonFX(Constants.RightClimbMotorCANID);

    LeftClimbMotor.configFactoryDefault();
    RightClimbMotor.configFactoryDefault();

    LeftClimbMotor.setInverted(false);
    RightClimbMotor.setInverted(true);

    LeftClimbMotor.setNeutralMode(NeutralMode.Brake);
    RightClimbMotor.setNeutralMode(NeutralMode.Brake);

    armsSolenoid = new DoubleSolenoid(Constants.PneumaticHub, PneumaticsModuleType.REVPH,
      Constants.ClimberArmsChannelFWD , Constants.ClimberArmsChannelREV);
    brakesSolenoid = new DoubleSolenoid(Constants.PneumaticHub, PneumaticsModuleType.REVPH,
      Constants.ClimberBrakesChannelFWD , Constants.ClimberBrakesChannelREV);

    resetClimbEncoders();
  }

  @Override
  public void periodic() {
    //UpdateDashboard();
  }

  // Motors
  public void ClimbUpTogether() {
    if ( (getRightClimbEncoder() < Constants.ClimberRightFullExtendEncoderReading) && (getLeftClimbEncoder() < Constants.ClimberLeftFullExtendEncoderReading) ) {
      RightClimbMotor.set(ControlMode.PercentOutput, Constants.ClimbMotorSpeed);
      LeftClimbMotor.set(ControlMode.PercentOutput, Constants.ClimbMotorSpeed);
    } else {
      stop();
    }
  }
  public void ClimbUp(){
    if (getRightClimbEncoder() < Constants.ClimberRightFullExtendEncoderReading) {
      // Encoder reading says we are okay to extend
      RightClimbMotor.set(ControlMode.PercentOutput, Constants.ClimbMotorSpeed);
    } else {
      RightClimbMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    if (getLeftClimbEncoder() < Constants.ClimberLeftFullExtendEncoderReading) {
      // Encoder reading says we are okay to extend
      LeftClimbMotor.set(ControlMode.PercentOutput, Constants.ClimbMotorSpeed);
    } else {
      LeftClimbMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void ClimbDown (){
    if (getRightClimbEncoder() > Constants.ClimberHomeEncoderReading) {
      // Encoder reading says we are okay to extend
      RightClimbMotor.set(ControlMode.PercentOutput, -Constants.ClimbMotorSpeed);
    } else {
      RightClimbMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    if (getLeftClimbEncoder() > Constants.ClimberHomeEncoderReading) {
      // Encoder reading says we are okay to extend
      LeftClimbMotor.set(ControlMode.PercentOutput, -Constants.ClimbMotorSpeed);
    } else {
      LeftClimbMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void stop(){
    RightClimbMotor.set(ControlMode.PercentOutput, 0);
    LeftClimbMotor.set(ControlMode.PercentOutput, 0);
  }
  public void ResetRightMotor() {
    // Run the LEFT climber motor down VERY SLOWLY to reset it's position after a match
    // Pit/test function only, this ignores the encoder reading
    RightClimbMotor.set(ControlMode.PercentOutput, -0.1);
  }
  public void ResetLeftMotor() {
    // Run the LEFT climber motor down VERY SLOWLY to reset it's position after a match
    // Pit/test function only, this ignores the encoder reading
    LeftClimbMotor.set(ControlMode.PercentOutput, -0.1);
  }

  // Solenoids/Pistons
  public void armsToggle(){
    if (armsSolenoid.get() == Value.kOff) {
      armsSolenoid.set(Value.kForward);
    } else {
      armsSolenoid.toggle();
    }
  }
  public void armsFWD(){
    armsSolenoid.set(Value.kForward);
  }
  public void armsREV(){
    armsSolenoid.set(Value.kReverse);
  }
  public void brakesOn(){
    brakesSolenoid.set(Value.kForward);
  }
  public void brakesOff(){
    brakesSolenoid.set(Value.kReverse);
  }
  
  // Sensors
  public double getLeftClimbEncoder(){
    return LeftClimbMotor.getSelectedSensorPosition();
  }
  public double getRightClimbEncoder(){
    return RightClimbMotor.getSelectedSensorPosition();
  }
  public void resetClimbEncoders(){
    LeftClimbMotor.setSelectedSensorPosition(0.0);
    RightClimbMotor.setSelectedSensorPosition(0.0);
  }

  // Smart Dashboard puts
  public void UpdateDashboard() {
    SmartDashboard.putNumber("LeftClimbEncoder", getLeftClimbEncoder());
    SmartDashboard.putNumber("RighgtClimbEncoder", getRightClimbEncoder());
    SmartDashboard.putBoolean("Cimber FWD", armsSolenoid.get() == Value.kForward);
    SmartDashboard.putBoolean("Brakes ON", brakesSolenoid.get() == Value.kForward);
  }
}
