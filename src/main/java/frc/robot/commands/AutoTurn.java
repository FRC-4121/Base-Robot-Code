/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.extraClasses.PIDControl;
import static frc.robot.Constants.DrivetrainConstants.*;


public class AutoTurn extends CommandBase {

  // Declare class variables
  double targetAngle;
	double startTime;
	double stopTime;
	double angleError;
	double angleCorrection;
	double motorOutput;

	PIDControl pidControl;

  private Timer timer = new Timer();
  
  private final Drivetrain drivetrain;



  /**
   * Class constructor
   */
  public AutoTurn(Drivetrain drive) {

    drivetrain = drive;

    //Use addRequirements() to require any subsystems for the command
    addRequirements(drivetrain);
                
    //Set up PID control
    pidControl = new PIDControl(kP_Turn, kI_Turn, kD_Turn);

  }


  /**
   * Called when the command is initially scheduled
   */
  @Override
  public void initialize() {

    timer.start();
		startTime = timer.get();
		angleError = 0;
		angleCorrection = 0;

  }


  /**
   * Called every time the scheduler runs while the command is scheduled
   */
  @Override
  public void execute() {

    angleCorrection = pidControl.run(drivetrain.getGyroAngle(), targetAngle);
		motorOutput = angleCorrection * kAutoTurnSpeed;
		drivetrain.autoDrive(motorOutput, -motorOutput);  


  }


  /**
   * Called once the command ends or is interrupted
   */
  @Override
  public void end(boolean interrupted) {

    drivetrain.stopDrive();

  }


  /**
   * Returns true when the command should end
   */
  @Override
  public boolean isFinished() {

    //Declare return flag
    boolean thereYet = false;

    //Check elapsed time
    if(stopTime<=timer.get()-startTime)
    {

      //Too much time has elapsed.  Stop this command
      thereYet = true;

    }
    else
    {

      angleError = drivetrain.getGyroAngle() - targetAngle;
      if (Math.abs(angleError) <= kTurnAngleTolerance)
      {
        
        thereYet = true;

      }

    }

    //Return the flag
    return thereYet;

  }


    /**
   * Set the parameters 
   */
  public void setParams(double angle, double time){

    //Set local variables
    targetAngle = angle;
    stopTime = time;

  }

}
