/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Turret;
import frc.robot.extraClasses.NetworkTableQuerier;
import frc.robot.extraClasses.PIDControl;


public class AimTurret extends CommandBase {

  //Declare class variables
  private Turret myTurret;
  private NetworkTableQuerier myNTQuerier;

  private boolean stopAutoTurret;
  private boolean overrideAutoTurret;
  private boolean foundTarget;
  private boolean targetLock;
  private boolean firstRun;
  private double targetOffset;
  private int targetLockCount;
  private boolean realTargetLock;

  private double turretCorrection;

  //Declare PID controller
  PIDControl lockPID;
  PIDControl turretPID;


  /**
   * Creates a new AimTurret.
   */
  public AimTurret(Turret turret, NetworkTableQuerier ntquerier) {

    // Assign variables
    myTurret = turret;
    myNTQuerier = ntquerier;

    // Declare subsystem dependencies
    addRequirements(myTurret);

  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Initialize flags
    stopAutoTurret = false;
    overrideAutoTurret = false;
    firstRun = true;
    targetLockCount = 0;
    realTargetLock = false;

    turretCorrection = 0;

    // Initialize PID
    lockPID = new PIDControl(kP_TurretLock, kI_TurretLock, kD_TurretLock);
    turretPID = new PIDControl(kP_Turret, kI_Turret, kD_Turret);


  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = 0;
    double turretAngle = myTurret.getTurretAngle();

    //If first time, rotate turret
    // if (firstRun){
    //   speed = -kTurretSpeed * myPID.run(turretAngle, 90.0);
    //   myTurret.rotateTurret(speed);
    //   if (turretAngle >= 85.0 && turretAngle <= 95.0){
    //     firstRun = false;
    //   }
    // }

    // Get status of flags/NT data
    foundTarget = myNTQuerier.getFoundTapeFlag();
    
    targetOffset = myNTQuerier.getTapeOffset();
    targetLock = (targetOffset < -5) && (targetOffset > -18);
    SmartDashboard.putBoolean("TargetLock", targetLock);

    //Need to update the override flag here (get value from gamepad switch)

    // Maintain target lock
    //If the driver has not overridden for manual control
    // if (!overrideAutoTurret){

      //If the camera has a target in sights
      if (foundTarget){

        // // Make sure we are really on the target
        // if (targetLock)
        // {
        //   targetLockCount++;
        // } else {
        //   targetLockCount = 0;
        // }

        // if (targetLockCount >= 10)
        // {
        //   realTargetLock = true;
        //   targetLockCount = 5;
        // }

        SmartDashboard.putNumber("LockCount", targetLockCount);

        //If the target is not centered in the screen
        if (!targetLock){

          //If the turret is in a safe operating range for the physical constraints of the robot
          // if (targetOffset > 0)
          // {
          //   speed = -kTurretSpeedLock;
          // } else {
          //   speed = kTurretSpeedLock;
          // }

          speed = -kTurretSpeedAuto * lockPID.run(targetOffset, -10);
          if (Math.abs(speed) > .10){
            if (speed < 0){
              speed = -.10;
            } else {
              speed = .10;
            }
          }
          // } else if (Math.abs(speed) < 0.06){
          //   if (speed < 0){
          //     speed = -0.06;
          //   }
          //   else{
          //     speed = 0.060;
          //   }
          // }
          SmartDashboard.putNumber("TurretSpeed", speed);
          

          myTurret.rotateTurret(speed);
        
        }
        else
        {
          //If target is locked, stop the motor
          myTurret.stopTurret();

          firstRun = false;
        }

      }
      else
      {
        //If the camera does not see a target, we need to figure out how to write the code for this
        // speed = -kTurretSpeedAuto * turretPID.run(turretAngle, 0.0);
        double targetAngle = 0;
        // if (Math.abs(turretAngle - targetAngle) < 2) {
        //   speed = 0;
        // }
        // else if (turretAngle - targetAngle > 2){
        //   speed = -kTurretSpeedLock;
        // }
        // else if (turretAngle - targetAngle < -2) {
        //   speed = kTurretSpeedLock;
        // }
        speed = 0;
        SmartDashboard.putNumber("TurretSpeed", speed);
        myTurret.rotateTurret(speed);
      }

    // }
    // else
    // {
    //   //Manual control will pull data from the potentiometer on the oi board.  For now, we will access a joystick as the 'potentiometer'
    // }

    
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myTurret.stopTurret();
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (realTargetLock) return true;
    else return false;
    
  }
}
