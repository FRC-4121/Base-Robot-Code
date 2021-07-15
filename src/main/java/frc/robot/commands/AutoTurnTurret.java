/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extraClasses.PIDControl;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.ShooterConstants.*;

public class AutoTurnTurret extends CommandBase {

  // Declare class variables
  double targetAngle;
	double startTime;
	double stopTime;
	double angleError;
	double angleCorrection;
	double speed;

	PIDControl pidControl;

  private Timer timer = new Timer();

  private Turret myTurret;


  /**
   * Class constructor
   * @param turret
   * @param angle
   * @param time
   */
  public AutoTurnTurret(Turret turret, double angle, double time) {

    myTurret = turret;

    targetAngle = angle;
    stopTime = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myTurret);

    //Set up PID control
    pidControl = new PIDControl(kP_Turret, kI_Turret, kD_Turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
		startTime = timer.get();
		angleError = 0;
		angleCorrection = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turretAngle = myTurret.getTurretAngle();
    angleCorrection = pidControl.run(turretAngle, targetAngle);
		speed = -kTurretSpeedAuto * angleCorrection;
    
    if(speed > 0){
      if(turretAngle <= kTurretMinAngle){
        myTurret.rotateTurret(speed);
      }
      else if(turretAngle < kTurretMaxAngle)
      {
        myTurret.rotateTurret(speed);
      }
      else
      {
        myTurret.stopTurret();
      }
    }
    else if (speed < 0)
    {
      if(turretAngle >= kTurretMaxAngle){
        myTurret.rotateTurret(speed);
      }
      else if(turretAngle > kTurretMinAngle){
        myTurret.rotateTurret(speed);
      }
      else
      {
        myTurret.stopTurret();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myTurret.stopTurret();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean thereYet = false;

    double time = timer.get();

    if (Math.abs(myTurret.getTurretAngle() - targetAngle) <= kTurretAngleTolerance){

      thereYet = true;

    }
    else if (stopTime <= time - startTime){

      thereYet = true;

    }

    return thereYet;
    
  }
}
