/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DrivetrainConstants.*;

public class DriveWithJoysticks extends CommandBase {
  
  private Drivetrain drivetrain;
  private XboxController xboxJoysticks;
  private Joystick lJoy, rJoy;
  private boolean useXboxControl;
  
  public DriveWithJoysticks(Drivetrain drive, XboxController xbox, Joystick leftJoy, Joystick rightJoy, boolean useXbox) {

    drivetrain = drive;
    xboxJoysticks = xbox;
    lJoy = leftJoy;
    rJoy = rightJoy;
    useXboxControl = useXbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(useXboxControl){
      //Drive using xbox joystick values
      double speedCorrection = kJoystickSpeedCorr;
      drivetrain.drive(speedCorrection * xboxJoysticks.getY(Hand.kLeft), kAutoRightSpeedCorrection * speedCorrection * xboxJoysticks.getY(Hand.kRight));
    }
    else
    {
      //Drive using joystick values (may need to be inverted)
      drivetrain.drive(lJoy.getY(), kAutoRightSpeedCorrection * rJoy.getY());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
