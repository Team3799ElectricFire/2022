// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

public class DriveTrain extends SubsystemBase {
  // Contents of the subsystem (Order is important!)
  private SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
  private Pigeon2 Gyro;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private double lastGyroReading, rotationalVelocity;
  private boolean DriveFieldRelative = false;
  private boolean TurnAroundCargo = false;
  private double SpeedMultiple = Constants.HighSpeedMultiple;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    // Create swerve modules
    frontLeftModule = new SwerveModule(Constants.frontLeftConstants);
    backLeftModule = new SwerveModule(Constants.backLeftConstants);
    backRightModule = new SwerveModule(Constants.backRightConstants);
    frontRightModule = new SwerveModule(Constants.frontRightConstants);
    resetEncoders();

    // Create gyro
    Gyro = new Pigeon2(Constants.PigeonGyroCANID);
    Pigeon2Configuration config = new Pigeon2Configuration();
    // set mount pose as rolled 90 degrees counter-clockwise
    config.MountPoseYaw = 0;
    config.MountPosePitch = 0;
    config.MountPoseRoll = 90;
    Gyro.configAllSettings(config);

    // Create kinematics library object
    // (remimber: +X = forward, +Y = left)
    Translation2d frontLefTranslation = new Translation2d(+.5* Constants.WheelBaseXMeters, +.5* Constants.WheelBaseYMeters);
    Translation2d backLefTranslation = new Translation2d(-.5* Constants.WheelBaseXMeters, +.5* Constants.WheelBaseYMeters);
    Translation2d backRightranslation = new Translation2d(-.5* Constants.WheelBaseXMeters, -.5* Constants.WheelBaseYMeters);
    Translation2d frontRightTranslation = new Translation2d(+.5* Constants.WheelBaseXMeters, -.5* Constants.WheelBaseYMeters);
    kinematics = new SwerveDriveKinematics(frontLefTranslation, backLefTranslation, backRightranslation, frontRightTranslation);

    // Create odometry object for tracking movement of robot around field
    odometry = new SwerveDriveOdometry(kinematics, GetGyroRot2d());
  }

  @Override
  public void periodic() {
    // This function is called repeatedly by the scheduler

    // Have the subsystem constantly polling the gyro to get rotational velocity, store the values
    double currentGyroReading = Gyro.getYaw(); // CCW is pos
    rotationalVelocity = currentGyroReading - lastGyroReading;
    // update last recorded reading
    lastGyroReading = currentGyroReading;

    // Update the odometry object to keep track of the robot moving around the field
    SwerveModuleState[] states = new SwerveModuleState[4];
    // remember order: frontLeft, backLeft, backRight, frontRight
    states[0] = frontLeftModule.getModuleState();
    states[1] = backLeftModule.getModuleState();
    states[2] = backRightModule.getModuleState();
    states[3] = frontRightModule.getModuleState();
    odometry.update(GetGyroRot2d(), states);

    //UpdateDashboard();
  }

  // Drive Functions
  public void DriveSwerve(double forward, double strafe, double rotation) {
    double vx, vy, omega;
    vx = forward * Constants.MaxSpeedMetersPerSec * SpeedMultiple;
    vy = strafe * Constants.MaxSpeedMetersPerSec * SpeedMultiple;
    omega = rotation * Constants.MaxTurnSpeedRadiansPerSec * SpeedMultiple;

    //System.out.println(String.format("FWD %.2f   |   STF %.2f   |   ROT %.2f",
    //vx, vy, omega));
    
    // Calls to kinematics library
    ChassisSpeeds Speed = new ChassisSpeeds(vx, vy, omega);

    //System.out.println(String.format("FWD %.2f   |   STF %.2f   |   ROT %.2f",
    //Speed.vxMetersPerSecond, Speed.vyMetersPerSecond, Speed.omegaRadiansPerSecond));

    Translation2d centerpoint = new Translation2d();
    if (TurnAroundCargo) {
      centerpoint = new Translation2d(Constants.CenterPointCargoXMeters, 0.0);
    }
    SwerveModuleState[] States = kinematics.toSwerveModuleStates(Speed, centerpoint);
    SwerveDriveKinematics.desaturateWheelSpeeds(States, Constants.MaxSpeedMetersPerSec);

    //System.out.println(String.format("FL %.2f   |   BL %.2f   |   BR %.2f   |   FR %.2f", 
    //States[0].angle.getDegrees(), States[1].angle.getDegrees(), States[2].angle.getDegrees(), States[3].angle.getDegrees()));
    
    // Send updates to swerve modules
    frontLeftModule.Update(States[0]);
    backLeftModule.Update(States[1]);
    backRightModule.Update(States[2]);
    frontRightModule.Update(States[3]);
  }
  public void DriveSwerveFieldRelative(double forward, double strafe, double rotation) {
    double vx, vy, omega, gyro;
    vx = forward * Constants.MaxSpeedMetersPerSec * SpeedMultiple;
    vy = strafe * Constants.MaxSpeedMetersPerSec * SpeedMultiple;
    omega = rotation * Constants.MaxTurnSpeedRadiansPerSec * SpeedMultiple;
    gyro = GetGyroDeg();

    // Calls to kinematics library
    ChassisSpeeds Speed = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(gyro));
    SwerveModuleState[] States = kinematics.toSwerveModuleStates(Speed);
    SwerveDriveKinematics.desaturateWheelSpeeds(States, Constants.MaxSpeedMetersPerSec);

    // Send updates to swerve modules
    frontLeftModule.Update(States[0]);
    backLeftModule.Update(States[1]);
    backRightModule.Update(States[2]);
    frontRightModule.Update(States[3]);
  }
  public void Stop() {
    frontLeftModule.Stop();
    backLeftModule.Stop();
    backRightModule.Stop();
    frontRightModule.Stop();
  }
  public void SetFullSpeed() {
    SpeedMultiple = Constants.HighSpeedMultiple;
  }
  public void SetHalfSpeed() {
    SpeedMultiple = Constants.LowSpeedMultiple;
  }
  public void SetTurboSpeed() {
    SpeedMultiple = Constants.TurboSpeedMultiple;
  }
  public void ToggleFieldRelative() {
    DriveFieldRelative^=true;
  }
  public boolean GetFieldRelative() {
    return DriveFieldRelative;
  }

  // Odometry
  public Pose2d getCurPose2d(){
    return odometry.getPoseMeters();
  }
  public void setCurPose2d(Pose2d pose) {
    odometry.resetPosition(pose, GetGyroRot2d());
  }

  // Sensors
  public void ZeroGyro() {
    Gyro.setYaw(0.0);
  }
  public void SetGyro(double newHeading) {
    Gyro.setYaw(newHeading);
  }
  public double GetGyroDeg() {
    return lastGyroReading; // Return the value fo the stored parameter instead of making another call to the gyro
  }
  public double GetGyroRad() {
    return Math.toRadians(GetGyroDeg());
  }
  public Rotation2d GetGyroRot2d() {
    return Rotation2d.fromDegrees(GetGyroDeg());
  }
  public double getRotationalVelocity(){
    return rotationalVelocity; // Return the value fo the stored parameter instead of making another call to the gyro
  }

  // Module Sensors
  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    backRightModule.resetEncoders();
    frontRightModule.resetEncoders();
  }
  public void zeroAllModulePosSensors() {
    frontLeftModule.zeroAbsPositionSensor();
    backLeftModule.zeroAbsPositionSensor();
    backRightModule.zeroAbsPositionSensor();
    frontRightModule.zeroAbsPositionSensor();
  }

  // Smart Dashboard puts
  public void UpdateDashboard() {
    // Report to driverstation
    SmartDashboard.putNumber("Drivetrain Gyro", GetGyroDeg());
    SmartDashboard.putNumber("FL Abs Angle", frontLeftModule.GetAbsSteerEncoderDeg());
    SmartDashboard.putNumber("BL Abs Angle", backLeftModule.GetAbsSteerEncoderDeg());
    SmartDashboard.putNumber("BR Abs Angle", backRightModule.GetAbsSteerEncoderDeg());
    SmartDashboard.putNumber("FR Abs Angle", frontRightModule.GetAbsSteerEncoderDeg());
    SmartDashboard.putNumber("Rot Velocity", rotationalVelocity);
    SmartDashboard.putNumber("Speed Multiple", SpeedMultiple);
    SmartDashboard.putBoolean("Field Relative Mode", DriveFieldRelative);
  }
}
