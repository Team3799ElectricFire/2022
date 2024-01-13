package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase{
    private final CANSparkMax FrontRevMotor, BackRevMotor, MidRevMotor;
    private final DoubleSolenoid theJawSolenoid; 
    private final DigitalInput BreakBeamFront, BreakBeamBack;

    public Intake(){
        FrontRevMotor = new CANSparkMax(Constants.FrontIntakeMotor, MotorType.kBrushless);
        BackRevMotor = new CANSparkMax(Constants.BackIntakeMotor, MotorType.kBrushless);
        MidRevMotor = new CANSparkMax(Constants.MidIntakeMotor, MotorType.kBrushless);

        theJawSolenoid = new DoubleSolenoid(Constants.PneumaticHub, PneumaticsModuleType.REVPH, Constants.IntakeJawChannelFWD, Constants.IntakeJawChannelREV);
        
        BreakBeamFront = new DigitalInput(Constants.BreakBeamID);
        BreakBeamBack = new DigitalInput(Constants.BreakBeam2ID);

        BackRevMotor.restoreFactoryDefaults();
        FrontRevMotor.restoreFactoryDefaults();
        MidRevMotor.restoreFactoryDefaults();
        
        BackRevMotor.setInverted(false);
        FrontRevMotor.setInverted(false);
        MidRevMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        //UpdateDashboard();
    }

    public void BackMotorOnly(){
        BackRevMotor.set(Constants.IntakeMotorSpeed);
    }

    public void IntakeCargo(){
        /*
        * Desired behavior: run intake motors in until break beam sensors detect cargo...
        * then stop the motor(s) so the cargo doesn't get sent to the launcher before the driver is ready
        */
        
        // back motor: run until back sensor is broken
        if (GetBreakBeamBack()) {
            // True means "I see something", so stop the motor
            BackRevMotor.stopMotor();
            FrontRevMotor.stopMotor();
            MidRevMotor.stopMotor();         
        } else {
            // False means no cargo yet, so run the motor
            BackRevMotor.set(Constants.IntakeMotorSpeed);
            FrontRevMotor.set(Constants.IntakeMotorSpeed);
            MidRevMotor.set(Constants.IntakeMotorSpeed);
        }

        /* Front beam sensor removed, we'll only hold 1 cargo now so all motors spind together based on only the back beam...
        *
        // front motor: run until both sensors are broken
        if (GetBreakBeamBack() && GetBreakBeamFront()) {
            // True means both sensors see a cargo, stop the motor
            FrontRevMotor.stopMotor();
            MidRevMotor.stopMotor();
        } else {
            // False means neither sensor, or only one sensor, sees a cargo, run the motor
            FrontRevMotor.set(Constants.IntakeMotorSpeed);
            MidRevMotor.set(Constants.IntakeMotorSpeed);
        }
        */
    }
    public void IntakeToLauncher(){
        FrontRevMotor.set(Constants.IntakeMotorSpeed);
        BackRevMotor.set(Constants.IntakeMotorSpeed);
        MidRevMotor.set(Constants.IntakeMotorSpeed);
    }
    public void IntakeStop(){
        FrontRevMotor.stopMotor();
        BackRevMotor.stopMotor();
        MidRevMotor.stopMotor();
    }
    public void IntakeReverse() {
        // Spit cargo back out the front of the robot
        FrontRevMotor.set(-Constants.IntakeMotorSpeed);
        BackRevMotor.set(-Constants.IntakeMotorSpeed);
        MidRevMotor.set(-Constants.IntakeMotorSpeed);
    }

    // Solenoid
    public void JawOpen(){
        //theJawSolenoid.getFwdChannel();
        theJawSolenoid.set(Value.kForward);
    }
    public void JawClose(){
        //theJawSolenoid.getRevChannel();
        theJawSolenoid.set(Value.kReverse);
    }
    public void JawOff(){
        theJawSolenoid.set(Value.kOff);
    }
    public void JawToggle(){
        if (theJawSolenoid.get() == Value.kOff) {
            JawClose();
        } else {
            theJawSolenoid.toggle();
        }
    }

    // Cargo sensors
    public boolean GetBreakBeamFront(){
        return !BreakBeamFront.get(); // Invert signal so "true" means "I see something"
    }
    public boolean GetBreakBeamBack(){
        return !BreakBeamBack.get(); // Invert signal so "true" means "I see something"
    }
    
    // Smart Dashboard puts
    public void UpdateDashboard() {
        SmartDashboard.putBoolean("FrontBeamReading", GetBreakBeamFront());
        SmartDashboard.putBoolean("BackBeamReading", GetBreakBeamBack());
    }
}
