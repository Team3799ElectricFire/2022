// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Auto_DelayShootThenBackOut;
import frc.robot.commands.Auto_DelayThenBackOut;
import frc.robot.commands.Auto_DriveSwerve;
import frc.robot.commands.Auto_FeedAFriend;
import frc.robot.commands.Auto_LowGoalThenBackOut;
import frc.robot.commands.Climber_Down;
import frc.robot.commands.Climber_Up;
import frc.robot.commands.Drive_ResetAllModulePositionsToZero;
import frc.robot.commands.Drive_Swerve;
import frc.robot.commands.Intake_CargoFromFloor;
import frc.robot.commands.Intake_CargoToLauncher;
import frc.robot.commands.Intake_ReverseCargo;
import frc.robot.commands.Launcher_EjectCargo;
import frc.robot.commands.Launcher_ReadyCannon;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // Create driverstation controller
  private final XboxController gamepad = new XboxController(0);
  private final JoystickButton aButton = new JoystickButton(gamepad, Constants.AButton);
  private final JoystickButton bButton = new JoystickButton(gamepad, Constants.BButton);
  private final JoystickButton xButton = new JoystickButton(gamepad, Constants.XButton);
  //private final JoystickButton yButton = new JoystickButton(gamepad, Constants.YButton);
  private final JoystickButton rbButton = new JoystickButton(gamepad, Constants.RBButton);
  private final JoystickButton lbButton = new JoystickButton(gamepad, Constants.LBButton);
  private final JoystickButton backButton = new JoystickButton(gamepad, Constants.BackButton);
  private final JoystickButton startButton = new JoystickButton(gamepad, Constants.StartButton);
  private final JoystickButton lsButton = new JoystickButton(gamepad, Constants.LSButton);
  private final JoystickButton rsButton = new JoystickButton(gamepad, Constants.RSButton);
  private final POVButton UpDPad = new POVButton (gamepad, 0);
  //private final POVButton LeftDPad = new POVButton(gamepad, 270);
  private final POVButton DownDPad = new POVButton (gamepad, 180);
  //private final POVButton RightDPad = new POVButton(gamepad, 90);
  private final JoystickAnalogButton RTrigger = new JoystickAnalogButton(gamepad, Constants.RTrigger);
  private final JoystickAnalogButton LTrigger = new JoystickAnalogButton(gamepad, Constants.LTrigger);

  // The robot's subsystems and commands are defined here...
  public final DriveTrain m_drivetrain = new DriveTrain();
  public final Climber m_climber = new Climber();
  public final Intake m_intake = new Intake();
  public final Launcher m_launcher = new Launcher();

  // Auto command selector (driverstation pickes which to run before each match)
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Assign default commands (these run when no buttons are pressed)
    m_drivetrain.setDefaultCommand(new Drive_Swerve(m_drivetrain, gamepad::getLeftY, gamepad::getLeftX, gamepad::getRightX));

    //Add some commands to dashboard for testing/configuring
    SmartDashboard.putData("CAL: Re-Cal Swerve Module Encoders", new Drive_ResetAllModulePositionsToZero(m_drivetrain)); // Puts a configuration command onto the dashboard
    SmartDashboard.putData("CAL: Reset Left Climb Motor", new InstantCommand(m_climber::ResetLeftMotor,m_climber));
    SmartDashboard.putData("CAL: Reset Right Climb Motor", new InstantCommand(m_climber::ResetRightMotor,m_climber));
    SmartDashboard.putData("CAL: Stop Climber Motors", new InstantCommand(m_climber::stop,m_climber));
    SmartDashboard.putData("DEBUG: Intake UP", new InstantCommand(m_intake::JawOpen,m_intake));
    SmartDashboard.putData("DEBUG: Intake DOWN", new InstantCommand(m_intake::JawClose,m_intake));
    SmartDashboard.putData("DEBUG: Climber Brakes ON", new InstantCommand(m_climber::brakesOn,m_climber));
    SmartDashboard.putData("DEBUG: Climber Brakes OFF", new InstantCommand(m_climber::brakesOff,m_climber));
    SmartDashboard.putData("DEBUG: Climber FWD", new InstantCommand(m_climber::armsFWD,m_climber));
    SmartDashboard.putData("DEBUG: Climber REV", new InstantCommand(m_climber::armsREV,m_climber));
    SmartDashboard.putData("DEBUG: Zero Gyro", new InstantCommand(m_drivetrain::ZeroGyro,m_drivetrain));
    SmartDashboard.putData("CAL: Zero Climber Encoders", new InstantCommand(m_climber::resetClimbEncoders,m_climber));
    configureAutoModes(); // Puts the autonomous mode picker onto the dashboard

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Assign commands to buttons
    /* Button Map
    * Lower Intake & get Cargo from Floor = RB √
    * Reverse Intake (spit out cargo)     = LB √
    * Full power shot (at goal)           = RT √
    * Low power shot (eject)              = LT √
    * Set target goal to low              = A √
    * Set target goal to high             = X √
    * Climber Extend                      = Dpad Up √
    * Climber Retract                     = Dpad Down √
    * Toggle Climber state (fwd/back)     = B √
    * Toggle Robot/Field Relative Drive   = Back √
    * Set drivetrain to 1/2 Speed         = LS √
    * Set drivetrain to full Speed        = RS √
    * Set drivetrain to turbo Speed       = Start √
    */
    rbButton.whenPressed(new SequentialCommandGroup(
      new InstantCommand(m_intake::JawClose, m_intake),
      new Intake_CargoFromFloor(m_intake)
      ));
    rbButton.whenReleased(new SequentialCommandGroup(
      new InstantCommand(m_intake::IntakeStop, m_intake),
      new InstantCommand(m_intake::JawOpen, m_intake)
      ));
    lbButton.whileHeld(new Intake_ReverseCargo(m_intake));
    //yButton.whenPressed(new InstantCommand(m_intake::JawToggle, m_intake));
    aButton.whenPressed(new InstantCommand(m_launcher::SetLowTarget, m_launcher));
    xButton.whenPressed(new InstantCommand(m_launcher::SetHighTarget, m_launcher));
    UpDPad.whileHeld(new Climber_Up(m_climber));
    DownDPad.whileHeld(new Climber_Down(m_climber));
    bButton.whenPressed(new InstantCommand(m_climber::armsToggle, m_climber));
    RTrigger.whenPressed(new SequentialCommandGroup(
      new Launcher_ReadyCannon(m_launcher),
      new Intake_CargoToLauncher(m_intake)
    ));
    RTrigger.whenReleased(new SequentialCommandGroup(
      new InstantCommand(m_launcher::CannonStop,m_launcher),
      new InstantCommand(m_intake::IntakeStop,m_intake)
    ));
    LTrigger.whenPressed(new SequentialCommandGroup(
      new Launcher_EjectCargo(m_launcher),
      new Intake_CargoToLauncher(m_intake)
    ));
    LTrigger.whenReleased(new SequentialCommandGroup(
      new InstantCommand(m_launcher::CannonStop,m_launcher),
      new InstantCommand(m_intake::IntakeStop,m_intake)
    ));
    lsButton.whenPressed(new InstantCommand(m_drivetrain::SetHalfSpeed,m_drivetrain));
    rsButton.whenPressed(new InstantCommand(m_drivetrain::SetFullSpeed,m_drivetrain));
    startButton.whenPressed(new InstantCommand(m_drivetrain::SetTurboSpeed,m_drivetrain));
    backButton.whenPressed(new InstantCommand(m_drivetrain::ToggleFieldRelative,m_drivetrain));
  }

  private void configureAutoModes() {
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1.0));

    // Add other autonomous options here...
    //autoChooser.addOption("Drive Forward", new <YourAutoCommand>);
    autoChooser.addOption("Drive Backwards 3sec", new Auto_DriveSwerve(m_drivetrain));
    autoChooser.addOption("Delay 7sec, Drive Backwards 3sec", new Auto_DelayThenBackOut(m_drivetrain, m_intake, 7.0, 3.0));
    autoChooser.addOption("Delay, Back up 1/2 way, Shoot, Back up rest of way", new Auto_DelayShootThenBackOut(m_drivetrain, m_intake, m_launcher, 7, 2.75));
    autoChooser.addOption("Shoot Low Goal, then back out", new Auto_LowGoalThenBackOut(m_drivetrain, m_intake, m_launcher));
    autoChooser.addOption("Feed Cargo to Ally, back away back/to the left", new Auto_FeedAFriend(m_drivetrain, m_intake));
    
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
    
  public void ZeroGyro() {
    m_drivetrain.ZeroGyro();
  }
}
