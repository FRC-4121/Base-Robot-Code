/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.extraClasses.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //Driver controllers
  private final XboxController xbox = new XboxController(XBOX_PORT);
  private final Joystick leftJoy = new Joystick(LEFT_JOYSTICK);
  private final Joystick testingJoystick = new Joystick(RIGHT_JOYSTICK);  

  //Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter();
  private final Turret turret = new Turret();
  private final Processor2 process2 = new Processor2();
  private final Pneumatics pneumatics = new Pneumatics();  
  private final CameraController camera = new CameraController();

  //Extra classes
  public final NetworkTableQuerier ntables = new NetworkTableQuerier();


  //Commands
  //Driving
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, xbox, leftJoy, testingJoystick, true);
  private final InvertDirection invertCommand = new InvertDirection(drivetrain, camera);

  //Pneumatics
  private final Shift shift = new Shift(pneumatics);
  private final OperateArm operateIntakeArm = new OperateArm(pneumatics);

  //Processor
  private final RunProcessor runProcessor = new RunProcessor(process2, false);
  private final RunProcessor invertProcessor = new RunProcessor(process2, true);

  //Shooter
  private final RunTurret clockwise = new RunTurret(turret, -kTurretSpeedManual);
  private final RunTurret counterclockwise = new RunTurret(turret, kTurretSpeedManual);
  private final RunShooter shoot = new RunShooter(shooter, testingJoystick);
  private final RaiseHood raiseHood = new RaiseHood(turret);
  private final LowerHood lowerHood = new LowerHood(turret);
  private final RunHoodToPos setNormalShoot = new RunHoodToPos(turret, 255);
  private final ShootWithSpeedConfirm shootSpeedConfirm = new ShootWithSpeedConfirm(shooter, process2);

  //Auto
  private final AimTurret aimTurret = new AimTurret(turret, ntables);
  private final ControlShooterSpeed autoShoot = new ControlShooterSpeed(shooter, ntables);
  

  //Buttons
  //Driving
  private final JoystickButton invertDirectionButton = new JoystickButton(xbox, 6);

  // Pneumatics
  private final JoystickButton shiftButton = new JoystickButton(xbox, 5);
  private final JoystickButton intakeArmButton = new JoystickButton(testingJoystick, 1);

  // Processor
  private final JoystickButton runProcButton = new JoystickButton(testingJoystick, 5);
  private final JoystickButton invertProcessorButton = new JoystickButton(testingJoystick, 6);

  // Shooter 
  private final JoystickButton clockwiseTurretButton = new JoystickButton(testingJoystick, 9);
  private final JoystickButton counterclockTurretButton = new JoystickButton(testingJoystick, 10);
  private final JoystickButton shootButton = new JoystickButton(testingJoystick, 11);
  private final JoystickButton testAutoTurret = new JoystickButton(testingJoystick, 12);
  private final JoystickButton hoodUp = new JoystickButton(testingJoystick, 7);
  private final JoystickButton hoodDown = new JoystickButton(testingJoystick, 8);
  private final JoystickButton hoodToNormal = new JoystickButton(xbox, 1);
  private final JoystickButton accuracyShoot = new JoystickButton(xbox, 2);

  // Extra Classes
  public final BallData ballData = new BallData();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    //Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

  }

  //For subsystem default commands (driving, etc.)
  private void configureDefaultCommands(){

    //Drivetrain -> drive with xbox joysticks
    drivetrain.setDefaultCommand(driveCommand);

    shooter.setDefaultCommand(shoot);//shoot: joystick control, autoShoot: automatic speed control

    // turret.setDefaultCommand(aimTurret);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Driving
    invertDirectionButton.whenPressed(invertCommand);
    
    //Pneumatics
    shiftButton.whenPressed(new InstantCommand(drivetrain::changeSpeed, drivetrain));
    intakeArmButton.whenPressed(operateIntakeArm);

    //Processor
    runProcButton.whileHeld(runProcessor);
    invertProcessorButton.whileHeld(invertProcessor);
    runProcButton.whenReleased(new InstantCommand(process2::stopProcessor, process2));
    invertProcessorButton.whenReleased(new InstantCommand(process2::stopProcessor, process2));
    
    //Shooter
    clockwiseTurretButton.whileHeld(clockwise);
    counterclockTurretButton.whileHeld(counterclockwise);
    clockwiseTurretButton.whenReleased(new InstantCommand(turret::stopTurret, turret));
    counterclockTurretButton.whenReleased(new InstantCommand(turret::stopTurret, turret));
    shootButton.whileHeld(new InstantCommand(process2::unlockProcessor, process2));
    shootButton.whenReleased(new InstantCommand(process2::stopProcessor, process2));
    hoodUp.whileHeld(raiseHood);
    hoodUp.whenReleased(new InstantCommand(turret::stopHood, turret));
    hoodDown.whileHeld(lowerHood);
    hoodDown.whenReleased(new InstantCommand(turret::stopHood, turret));
    testAutoTurret.whenPressed(aimTurret);
    hoodToNormal.whenPressed(setNormalShoot);
    accuracyShoot.whenPressed(shootSpeedConfirm);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. 
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoDriveToBall(drivetrain, pneumatics, ntables, 20);
    // return new AutoGetAllBalls(drivetrain, pneumatics, process2, ntables, ballData, 2, 100);
    // return new AutoShootTimed(drivetrain, shooter, pneumatics, process2, turret, ntables, 60);
    // return new RunHoodToPos(turret, 240);
  }


  /**
   * Return the Auto Shoot command
   * @return
   */
  public Command getAutoShootCommand(){

    return autoShoot;

  } 
}
