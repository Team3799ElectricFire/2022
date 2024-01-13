
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX _driveMotor, _steerMotor;
    private final CANCoder _steerEncoder;
    private final SwerveModuleConstants _Constants;
    
    public static class SwerveModuleConstants {
        public int SteerEncoderCANID = 0;
        public int DriveMotorCANID = 1;
        public int SteerMotorCANID = 2; 
        public double SteerEncoderOffsetDeg = 0;
        public double WheelDiameterMeters = 0.1016; // m = 0.1016m = 4in (or Freedom Units) 
        
        public double DriveWheelKp = 0.1;
        public double DriveWheelKi = 0.001;
        public double DriveWheelKd = 5;
        public double DriveWheelKf = 1023/21932; // 1023 represents output value to Talon at 100%, 21932 represents Velocity units at 100% output
        public boolean DriveInverted = false;
        
        public double SteerMotorKp = 0.035;//0.5;
        public double SteerMotorKi = 0.00001;
        public double SteerMotorKd = 0.001;
        public double SteerMotorKf = 0.0;
        public double SteerMotorIzone = 0;
        public double SteerMotorCV = 1700;
        public double SteerMotorAcc = SteerMotorCV * 12;
        public double SteerMotorAE = 0.1;
    }

    public SwerveModule(SwerveModuleConstants SwerveConstants) {
        _Constants = SwerveConstants;

        // Set up Steering Encoder.
        _steerEncoder = new CANCoder(SwerveConstants.SteerEncoderCANID);
        //_steerEncoder.configFactoryDefault();
        _steerEncoder.configSensorDirection(false);
        _steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        _steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
        _steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 333);

        // Setup Drive Motor.
        _driveMotor = new TalonFX(SwerveConstants.DriveMotorCANID);
        _driveMotor.configFactoryDefault();
        _driveMotor.setInverted(SwerveConstants.DriveInverted);
        _driveMotor.setNeutralMode(NeutralMode.Brake);
        _driveMotor.config_kP(0, SwerveConstants.DriveWheelKp);
        _driveMotor.config_kI(0, SwerveConstants.DriveWheelKi);
        _driveMotor.config_kD(0, SwerveConstants.DriveWheelKd);
        _driveMotor.config_kF(0, SwerveConstants.DriveWheelKf);
        _driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        _driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        // Setup Steering Motor.
        _steerMotor = new TalonFX(SwerveConstants.SteerMotorCANID);
        _steerMotor.configFactoryDefault();
        _steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _steerMotor.setInverted(true);
        _steerMotor.setNeutralMode(NeutralMode.Brake);
        _steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
        _steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
        _steerMotor.config_kP(0, SwerveConstants.SteerMotorKp);
        _steerMotor.config_kI(0, SwerveConstants.SteerMotorKi);
        _steerMotor.config_kD(0, SwerveConstants.SteerMotorKd);
        _steerMotor.config_kF(0, SwerveConstants.SteerMotorKf);
        _steerMotor.configAllowableClosedloopError(0,SwerveConstants.SteerMotorAE);
    }

    // Driving Functions
    public void Update(SwerveModuleState target){

        // Current rotation
        Rotation2d curPosition = getAbsSteerEncoderRotation2d();

        // Optimize target state with Kinematics library
        SwerveModuleState optimizedTarget = Optimize(target, curPosition);

        // Corrections for -180/+180 discontinuity
        double posDiff = optimizedTarget.angle.getRadians() - curPosition.getRadians();
        double absDiff = Math.abs(posDiff);

        // If the distance is more than half a circle, we are going the wrong way
        if (absDiff > Math.PI) {
            posDiff = posDiff - (2*Math.PI * Math.signum(posDiff));
        }

        // Convert the shortest distance of rotation to relative encoder value(use convertion factor)
        double targetAngle = RadiansToFalconCounts(posDiff);
        double outputEncoderValue = targetAngle + getRelSteerEncoder();

        
        // Don't command this module to move if its' velocity is too small
        if (Math.abs(optimizedTarget.speedMetersPerSecond) < Constants.MinSpeedMetersPerSec) {
            Stop();
        } else {
            // Set steering motor
            _steerMotor.set(TalonFXControlMode.Position, outputEncoderValue);

            // Set drive motor
            _driveMotor.set(TalonFXControlMode.Velocity, MetersPerSecondToVelocityUnits(optimizedTarget.speedMetersPerSecond));
        }

        /* // Set steering motor
        _steerMotor.set(TalonFXControlMode.Position, outputEncoderValue);
        
        // Don't command this module to move if its' velocity is too small
        if (Math.abs(optimizedTarget.speedMetersPerSecond) > Constants.MinSpeedMetersPerSec) {
             _driveMotor.set(TalonFXControlMode.Velocity, MetersPerSecondToVelocityUnits(optimizedTarget.speedMetersPerSecond));
        } else {
            _driveMotor.set(TalonFXControlMode.Velocity, 0.0);
        } */
    }
    public void Stop() {
        _driveMotor.set(ControlMode.PercentOutput,0.0);
        _steerMotor.set(ControlMode.PercentOutput,0.0);
    }
    /**
    * Minimize the change in heading the desired swerve module state would require by potentially
    * reversing the direction the wheel spins. If this is used with the PIDController class's
    * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
    *
    * @param desiredState The desired state.
    * @param currentAngle The current module angle.
    * @return Optimized swerve module state.
    */
    public static SwerveModuleState Optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getRadians()) > Constants.PI_OVER_TWO) {
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Constants.ROTATE_BY_PI));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    // Sensors
    public double GetAbsSteerEncoderDeg() {
        return _steerEncoder.getAbsolutePosition();
    }
    public Rotation2d getAbsSteerEncoderRotation2d () {
        return Rotation2d.fromDegrees(GetAbsSteerEncoderDeg());
    }
    public double getRelSteerEncoder() {
        return _steerMotor.getSelectedSensorPosition();
    }
    public double getSpeedMetersPerSec() {
        return VelocityUnitsToMetersPerSec(_driveMotor.getSelectedSensorVelocity());
    }
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getSpeedMetersPerSec(), getAbsSteerEncoderRotation2d());
    }
    public void resetEncoders() {
        _driveMotor.setSelectedSensorPosition(0);
        _steerMotor.setSelectedSensorPosition(0);
    }
    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to set the offset of the 
     * CANCoder so we can dictate the zero position. 
     * INPUTS MUST BE IN DEGREES. 
     * 
     * @param value a number between -180 and 180, where 0 is straight ahead
     */
    private void setRotateAbsSensor(double value) {
        _steerEncoder.configMagnetOffset(value, 0);
    }
    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to change the offset of 
     * the CANCoder so we dictate the zero position as the 
     * current position of the module.
     */
    public void zeroAbsPositionSensor() {
        //find the current offset, subtract the current position, and makes this number the new offset.
        setRotateAbsSensor(this._steerEncoder.configGetMagnetOffset()-GetAbsSteerEncoderDeg());
    }


    // Unit Conversion Math
    public double MetersPerSecondToVelocityUnits(double MetersPerSecond){
        // 2048 = Encoder Counts / 1rev (Talon FX internal encoder)
        // 7.85 = Gear Ratio
        // 10 = Unit Conversion to msec
        return MetersPerSecond * (2048 * 7.85) / (Math.PI * _Constants.WheelDiameterMeters * 10);
    }
    public double VelocityUnitsToMetersPerSec(double Velocity) {
        // 2048 = Encoder Counts / 1rev (Talon FX internal encoder)
        // 7.85 = Gear Ratio
        // 10 = Unit Conversion to msec
        return Velocity * (Math.PI * _Constants.WheelDiameterMeters * 10) / (2048 * 7.85);
    }

    public double FalconCountsToRadians(double Counts) {
        // 2048 = Encoder Counts / 1rev (Talon FX internal encoder)
        // 15.43 = gear ratio b/w talon and steering (Motor -> 8:24 * 14:72 -> Wheel)
        return (Counts / (2048*15.43)) * (2*Math.PI);
    }
    public double RadiansToFalconCounts(double Radians) {
        // 2048 = Encoder Counts / 1rev (Talon FX internal encoder)
        // 15.43 = gear ratio b/w talon and steering (Motor -> 8:24 * 14:72 -> Wheel)
        return Radians * (2048*15.43) / (2*Math.PI);
    }
    public double FalconCountsToDegrees(double Counts) {
        // 2048 = Encoder Counts / 1rev (Talon FX internal encoder)
        // 15.43 = gear ratio b/w talon and steering (Motor -> 8:24 * 14:72 -> Wheel)
        return (Counts / (2048*15.43)) * (360);
    }
    public double DegreesToFalconCounts(double Degrees) {
        // 2048 = Encoder Counts / 1rev (Talon FX internal encoder)
        // 15.43 = gear ratio b/w talon and steering (Motor -> 8:24 * 14:72 -> Wheel)
        return Degrees * (2048*15.43) / (360);
    }
}