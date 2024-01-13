// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SwerveModule.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /* Factors of PI */
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;
    public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);

    //Swerve Modules for drivetrain
    public static final int PigeonGyroCANID = 38;
    public static final SwerveModuleConstants frontLeftConstants = new SwerveModuleConstants();
    static {
        frontLeftConstants.SteerEncoderCANID = 03;
        frontLeftConstants.DriveMotorCANID = 01;
        frontLeftConstants.SteerMotorCANID = 02;
        frontLeftConstants.SteerEncoderOffsetDeg = 0;
    }
    public static final SwerveModuleConstants backLeftConstants = new SwerveModuleConstants();
    static {
        backLeftConstants.SteerEncoderCANID = 13;
        backLeftConstants.DriveMotorCANID = 11;
        backLeftConstants.SteerMotorCANID = 12;
        backLeftConstants.SteerEncoderOffsetDeg = 0;
    }
    
    public static final SwerveModuleConstants backRightConstants = new SwerveModuleConstants();
    static {
        backRightConstants.SteerEncoderCANID = 23;
        backRightConstants.DriveMotorCANID = 21;
        backRightConstants.SteerMotorCANID = 22;
        backRightConstants.SteerEncoderOffsetDeg = 0;
    }
    public static final SwerveModuleConstants frontRightConstants = new SwerveModuleConstants();
    static {
        frontRightConstants.SteerEncoderCANID = 33;
        frontRightConstants.DriveMotorCANID = 31;
        frontRightConstants.SteerMotorCANID = 32;
        frontRightConstants.SteerEncoderOffsetDeg = 0;
    }
    
    //pneumatics
    public static final int PneumaticHub = 41;
    
    // Drivetrain Size
    public static final double WheelBaseYMeters = 21.25 / 39.37; // 21.25 inches
    public static final double WheelBaseXMeters = 24.25 / 39.37; // 24.25 inches
    public static final double CenterPointCargoXMeters = 0.3048 + 0.5*WheelBaseXMeters; // 10 inches in front of front modules

    // Drivetrain speeds
    public static final double MaxSpeedMetersPerSec = 4.325;
    public static final double MinSpeedMetersPerSec = 0.01;
    public static final double MaxTurnSpeedRadiansPerSec = 3.542;
    public static final double TurboSpeedMultiple = 0.75;
    public static final double HighSpeedMultiple = 0.55;
    public static final double LowSpeedMultiple = 0.25;

    //Climber ID and constants
    public static final int LeftClimbMotorCANID = 7;
    public static final int RightClimbMotorCANID = 27;
    public static final double ClimbMotorSpeed = 0.75;
    public static final int ClimberArmsChannelFWD = 6;
    public static final int ClimberArmsChannelREV = 1;
    public static final int ClimberBrakesChannelFWD = 5;
    public static final int ClimberBrakesChannelREV = 2;
    public static final int ClimberLeftFullExtendEncoderReading = 360000; // encoder counts (of max 388000)
    public static final int ClimberRightFullExtendEncoderReading = 400000; // encoder counts (of max 422000)
    public static final int ClimberHomeEncoderReading = 500; // encoder counts


    // intake ID and constants
    public static final int FrontIntakeMotor = 26;
    public static final int BackIntakeMotor = 25;
    public static final int MidIntakeMotor = 24;
    public static final int IntakeSolenoid = 10;
    public static final int BreakBeamID = 8;
    public static final int BreakBeam2ID = 9;
    public static final double IntakeMotorSpeed = .75;
    public static final int IntakeJawChannelFWD = 3;
    public static final int IntakeJawChannelREV = 0;

    //Launcher
    public static final int LeftLauncherMotorCANID = 14;
    public static final int RightLauncherMotorCANID = 15;
    // MAX VELOCITY = 20660 encoder counts per 100ms
    public static final double LauncherHighSpeed = 12000; // encoder counts per 100ms (for velocity mode)
    public static final double LauncherLowSpeed = 9000; // encoder counts per 100ms (for velocity mode)
    public static final double LauncherEjectSpeed = 5000; // encoder counts per 100ms (for velocity mode)


    // Controller Buttons
    public static final int AButton = 1;
    public static final int BButton = 2;
    public static final int XButton = 3;
    public static final int YButton = 4;
    public static final int LBButton = 5;
    public static final int RBButton = 6;
    public static final int BackButton = 7;
    public static final int StartButton = 8;
    public static final int LSButton = 9;
    public static final int RSButton = 10;
    public static final int RTrigger = 3;
    public static final int LTrigger = 2;
    
    //public static final double gamepad = 0;
    public static final double ThumbstickDeadBand = 0.1;


}
