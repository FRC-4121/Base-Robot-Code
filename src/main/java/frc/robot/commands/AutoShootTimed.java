// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Processor2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.extraClasses.NetworkTableQuerier;
import frc.robot.extraClasses.PIDControl;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.extraClasses.Ballistics;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.ShooterConstants.*;


/**
 * AutoShootTimed Command
 * 
 * This command repeatedly drives the robot between a shooting location
 * and a ball loading location.  At the shooting location, the robot shoots
 * three balls at the goal.  At the loading location, the robot
 * waits for three balls to be loaded into the processor before
 * moving back to the shooting location.
 */
public class AutoShootTimed extends CommandBase {

  // Declare class variables
  private final Drivetrain myDrivetrain;
  private final Shooter myShooter;
  private final Processor2 myProcessor;
  private final Turret myTurret;
  private final Pneumatics myPneumatics;
  private final NetworkTableQuerier myNTables;
  private Ballistics myBallistics;

  private int robotMode;      // 1: Shooting, 2: Drive to Loading, 3: Loading Wait, 4: Drive to Shooting
  private int ballCount;

  private double driveDistance;
  private double targetDriveDistance;
  private double driveDirection;
  private double driveSpeedCorrection;
  private double driveSpeed;
  private double leftEncoderStart;
  private double rightEncoderStart;
  private double totalRotationsRight;
  private double totalRotationsLeft;
  private double stopTime;
  private double startTime;
  private double currentGyroAngle;
  private double angleCorrection;
  private double angleDeadband;
  private double targetOffset;
  private double turretCorrection;
  private double targetDistance;
  private double targetShooterSpeed;
  private double targetShooterSpeedRPM;
  private double targetShooterSpeedCorrected;
  private double shooterSpeed;
  private double shooterSpeedCorrection;
  private double shotPossible;//Ballistics value; 0 is false, 1 is 
  private double shotWaitTime;
  private double shotTime;
  private double loopCount = 0;
  private boolean shooting = false;
  private boolean ballEntering;
  private boolean turretLocked;
  private boolean timeSet;
  private double allShotsTime;
  private double lastShotWaitTime;
  private double targetAngle;
  private int targetLockCount;
  private double returnDriveDistance;
  private int ballPresentCount = 0;

  private boolean foundTarget;
  private boolean targetLock;
  private boolean realTargetLock;
  private boolean runSpeedControl;

  private double[] ballisticsData;

  private Timer runTimer;
  private Timer shotTimer;

  private PIDControl pidDriveAngle;
  private PIDControl pidDriveDistance;
  private PIDControl pidShooterSpeed;
  private PIDControl pidLock;
  private PIDControl pidTurret;


  /** Default constructor */
  public AutoShootTimed(Drivetrain drive, Shooter shoot, Pneumatics pneumatics, Processor2 process, Turret shootturret, NetworkTableQuerier table, double time) {

    // Set class variables
    myDrivetrain = drive;
    myShooter = shoot;
    myProcessor = process;
    myTurret = shootturret;
    myPneumatics = pneumatics;
    myNTables = table;
    stopTime = time;

    // Add subsystem requirements
    addRequirements(myDrivetrain, myShooter, myProcessor, myTurret);

    // Create PID controllers
    pidDriveAngle = new PIDControl(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);
    pidDriveDistance = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    pidShooterSpeed = new PIDControl(kP_Shoot, kI_Shoot, kD_Shoot);
    pidLock = new PIDControl(kP_TurretLock, kI_TurretLock, kD_TurretLock);
    pidTurret = new PIDControl(kP_Turret, kI_Turret, kD_Turret);

    // Create the ballistics table
    myBallistics = new Ballistics(98.25, 22.5, 5, 6050, 6, .25);

  }


  /** Initialize this command (only runs first time command is called) */
  @Override
  public void initialize() {

    // Initialize variables
    driveDirection = -1;
    driveDistance = 0;
    driveSpeed = kAutoDriveSpeed;
    targetDriveDistance = 90;
    returnDriveDistance = 100;
    driveSpeedCorrection = 1;
    totalRotationsRight = 0;
    totalRotationsLeft = 0;
    ballCount = 3;
    turretCorrection = 0;
    shooterSpeed = .75;
    shooterSpeedCorrection = 0;
    targetShooterSpeed = kShooterMaxRPM;
    targetShooterSpeedRPM = kShooterMaxRPM;
    targetDistance = 0;
    shotWaitTime = .25;
    ballEntering = false;
    turretLocked = false;
    targetAngle = 10.4;
    targetLockCount = 0;
    realTargetLock = false;
    
    timeSet = false;
    lastShotWaitTime = .25;

    // Initialize flags
    runSpeedControl = true;

    // Initialize shooting mode
    robotMode = 1;

    // Start the timers and get the command start time
    runTimer = new Timer();
    runTimer.start();
    startTime = runTimer.get();

    shotTimer = new Timer();
    shotTimer.start();
    shotTime = shotTimer.get();
    
    // Zero gyro angle
    myDrivetrain.zeroGyro();

    // Get starting encoder positions
    leftEncoderStart = myDrivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = myDrivetrain.getMasterRightEncoderPosition();

    myPneumatics.retractIntake();

  }


  /** Main code to be executed every robot cycle */
  @Override
  public void execute() {

    // Get status of flags/NT data
    foundTarget = myNTables.getFoundTapeFlag();
    // targetLock = myNTables.getTargetLockFlag();
    
    targetOffset = myNTables.getTapeOffset();
    targetLock = (targetOffset < -8) && (targetOffset > -18);//adjustment for testing
    
    SmartDashboard.putBoolean("TargetLock", targetLock);
    targetDistance = myNTables.getTapeDistance();


    // Aim turret and configure hood position
    double turretSpeed = 0;
    double turretAngle = myTurret.getTurretAngle();
    SmartDashboard.putBoolean("TurretLocked", turretLocked);
    if (robotMode == 1){
      
      if (foundTarget) {  
      
        // Make sure we are really on the target
        // if (targetLock)
        // {
        //   targetLockCount++;
        // } else {
        //   targetLockCount = 0;
        // }

        // if (targetLockCount >= 3)
        // {
        //   realTargetLock = true;
        //   targetLockCount = 3;
        // }

        //If the target is not centered in the screen
        if (!targetLock) {

          //If the turret is in a safe operating range for the physical constraints of the robot
          turretSpeed = -kTurretSpeedAuto * pidLock.run(targetOffset, -10);
          // speed = -kTurretSpeedAuto * lockPID.run(targetOffset, 0);
          if (Math.abs(turretSpeed) > .10){
            if (turretSpeed < 0){
              turretSpeed = -.10;
            } else {
              turretSpeed = .10;
            }
          }
          // if (targetOffset > 0)
          // {
          //   turretSpeed = -kTurretSpeedLock;
          // } else {
          //   turretSpeed = kTurretSpeedLock;
          // }
          SmartDashboard.putNumber("TurretSpeed", turretSpeed);

          myTurret.rotateTurret(turretSpeed);
        
        } else {

          //If target is locked, stop the motor
          myTurret.stopTurret();
          targetAngle = myTurret.getTurretAngle();

        }
      
      } else {
         //If the camera does not see a target, we need to figure out how to write the code for this
        // turretSpeed = -kTurretSpeedAuto * pidTurret.run(turretAngle, 0.0);
        // SmartDashboard.putNumber("TurretSpeed", turretSpeed);
        // myTurret.rotateTurret(turretSpeed);
        myTurret.stopTurret();
      }
      
    } else {
      
      // turretSpeed = -kTurretSpeedAuto * pidTurret.run(turretAngle, targetAngle);
      // SmartDashboard.putNumber("TurretSpeed", turretSpeed);
      // myTurret.rotateTurret(turretSpeed);
      turretSpeed = 0;
      // if (Math.abs(turretAngle - targetAngle) < 2) {
      //   turretSpeed = 0;
      // }
      // else if (turretAngle - targetAngle > 2){
      //   turretSpeed = -kTurretSpeedLock;
      // }
      // else if (turretAngle - targetAngle < -2) {
      //   turretSpeed = kTurretSpeedLock;
      // }
      SmartDashboard.putNumber("TurretSpeed", turretSpeed);
      // myTurret.rotateTurret(turretSpeed);
      myTurret.stopTurret();

    }

    targetShooterSpeedCorrected = 0;

    // Control shooter speed
    if(runSpeedControl) {

      // targetLock = myNTables.getTargetLockFlag();

      if(targetLock) {
        
        // distance = 153;
        ballisticsData = myBallistics.queryBallisticsTable(targetDistance);
        shotPossible = ballisticsData[0];

        if(shotPossible == 0){

          SmartDashboard.putBoolean("Shot Possible", false);
          targetShooterSpeed = shooterSpeed;

        } else {

          SmartDashboard.putBoolean("Shot Possible", true);
          targetShooterSpeed = ballisticsData[2];

        }

        // Calculate speed correction
        
        SmartDashboard.putNumber("BallisticsRPM", targetShooterSpeed * kShooterMaxRPM);
        shooterSpeedCorrection = pidShooterSpeed.run(myShooter.getShooterRPM(), targetShooterSpeed*kShooterMaxRPM);
        
        // Correct shooter speed control input
        //targetShooterSpeedCorrected = targetShooterSpeed * kSpeedCorrectionFactor;
        targetShooterSpeedCorrected = targetShooterSpeed - shooterSpeedCorrection;

        // Ensure corrected speed is within bounds
        if (targetShooterSpeedCorrected > 1) {
          targetShooterSpeedCorrected = 1;
        } else if (targetShooterSpeedCorrected < -1) {
          targetShooterSpeedCorrected = -1;
        }

        // Write key values to dashboard
        SmartDashboard.putNumber("Ballistics Speed", targetShooterSpeedCorrected);
        SmartDashboard.putNumber("Shooter Speed Correct", shooterSpeedCorrection);

        myShooter.shoot(targetShooterSpeed - shooterSpeedCorrection);
        //myShooter.shoot(targetShooterSpeedCorrected);
        //I have battery concerns about this implementation.  If we notice that battery draw during a match is problematic for speed control, we
        //will need to revert to a pid for RPM in some way.  This would be sufficiently complicated that it is a low priority, however.

      } else {

        myShooter.shoot(shooterSpeed);

      }

    } else {

      myShooter.shoot(shooterSpeed);  

    } 


    // Determine mode and take action
    switch(robotMode) {

      // Shooting
      case 1:

        //Run processor normally regardless of position conditions
        myProcessor.lockProcessor();
        myProcessor.autoRunProcessor(false, true);

        if (targetLock) {

          //Ensure wheel is moving fast enough to accurately make shot
          double l_targetSpeed = targetShooterSpeed * kShooterMaxRPM;
          SmartDashboard.putNumber("l_targetSpeed", l_targetSpeed);
          SmartDashboard.putNumber("Shooter RPM", myShooter.getShooterRPM());
          SmartDashboard.putNumber("tolerance", kRPMTolerance);
          
          if (ballCount > 0) {
            double time = shotTimer.get();
            SmartDashboard.putNumber("Shot Timer", time - shotTime);
            if(time - shotTime >= shotWaitTime){
              if (Math.abs(Math.abs(myShooter.getShooterRPM()) - targetShooterSpeed * kShooterMaxRPM) < kRPMTolerance || shooting) {
                myProcessor.unlockProcessor();
                shooting = true;
                loopCount++;
                SmartDashboard.putNumber("LoopCount", loopCount);
                if (loopCount == 8) {
                  ballCount--;
                  shotTime = time;
                  loopCount = 0;
                  shooting = false;
                }
              }  
            }
          } 

        }

        break;
      
      // Driving to load area
      case 2:

        // Calculate angle correction based on gyro reading
        currentGyroAngle = myDrivetrain.getGyroAngle();
        angleCorrection = pidDriveAngle.run(currentGyroAngle, 0);
        SmartDashboard.putNumber("AngleCor", angleCorrection);

        // Calculate speed correction based on distance to target
        totalRotationsRight = Math.abs(myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart);
        totalRotationsLeft = Math.abs(myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart);
        driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
        // driveSpeedCorrection = pidDriveDistance.run(driveDistance, targetDriveDistance);
        // SmartDashboard.putNumber("DrSpCorrection", driveSpeedCorrection);

        // //Check for overspeed correction
        // if(driveSpeedCorrection > 1){

        //   driveSpeedCorrection = 1;

        // } else if (driveSpeedCorrection < -1) {

        //   driveSpeedCorrection = -1;

        // }

        // Calculate final drive speed
        driveSpeedCorrection = 1;
        SmartDashboard.putNumber("DistanceToGo", Math.abs(driveDistance - targetDriveDistance));
        if (Math.abs(driveDistance - targetDriveDistance) <= 20) {
          driveSpeedCorrection = 0.75;
          // angleCorrection = 0;
        }
        driveDirection = -1;
        driveSpeed = driveDirection * driveSpeedCorrection * kAutoShootDriveSpeed;

        // // Enforce minimum speed
        // if (Math.abs(driveSpeed) < kAutoDriveSpeedMin) {

        //   angleCorrection = 0;
        //   if (driveSpeed < 0){
        //     driveSpeed = -kAutoDriveSpeedMin;
        //   } else {
        //     driveSpeed = kAutoDriveSpeedMin;
        //   }
        // }

        double leftSpeed = 0;
        double rightSpeed = 0;
        if (Math.abs(driveSpeed + angleCorrection) > 1){
          if(driveSpeed + angleCorrection < 0) {
            leftSpeed = -1;
          } else {
            leftSpeed = 1;
          }
        } else {
          leftSpeed = driveSpeed + angleCorrection;
        }

        angleCorrection = 0;
        if (Math.abs(driveSpeed - angleCorrection) > 1){
          if(driveSpeed - angleCorrection < 0) {
            rightSpeed = -1;
          } else {
            rightSpeed = 1;
          }
        } else {
          rightSpeed = driveSpeed - angleCorrection;
        }
        rightSpeed *= kAutoRightSpeedCorrection;
        SmartDashboard.putNumber("LeftSpeed", leftSpeed);
        SmartDashboard.putNumber("RightSpeed", rightSpeed);
    
        // Run the drive
        myDrivetrain.autoDrive(leftSpeed, rightSpeed);
        break;
      
      // Waiting in load area
      case 3:

        //Run the processor continually
        myProcessor.autoRunProcessor(false, true);

        break;
      
      // Driving to shooting location
      case 4:

        // Calculate angle correction based on gyro reading
        currentGyroAngle = myDrivetrain.getGyroAngle();
        angleCorrection = pidDriveAngle.run(currentGyroAngle, 0);

        // Calculate speed correction based on distance to target
        totalRotationsRight = Math.abs(myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart);
        totalRotationsLeft = Math.abs(myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart);
        driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
        // driveSpeedCorrection = pidDriveDistance.run(driveDistance, targetDriveDistance);

        // //Check for overspeed correction
        // if(driveSpeedCorrection > 1){

        //   driveSpeedCorrection = 1;
        
        // } else if (driveSpeedCorrection < -1) {
        
        //   driveSpeedCorrection = -1;
        
        // }

        // Calculate final drive speed
        driveSpeedCorrection = 1;
        if (Math.abs(driveDistance - (returnDriveDistance)) <= 20) {
          driveSpeedCorrection = 0.75;
          // angleCorrection = 0;
        }
        driveDirection = 1;
        driveSpeed = driveDirection * driveSpeedCorrection * kAutoShootDriveSpeed;

        // Enforce minimum speed
        // if (Math.abs(driveSpeed) < kAutoDriveSpeedMin) {

        //   if (driveSpeed < 0){
        //     driveSpeed = -kAutoDriveSpeedMin;
        //   } else {
        //     driveSpeed = kAutoDriveSpeedMin;
        //   }
        // }
        angleCorrection = 0;
        leftSpeed = 0;
        rightSpeed = 0;
        if (Math.abs(driveSpeed + angleCorrection) > 1){
          if(driveSpeed + angleCorrection < 0) {
            leftSpeed = -1;
          } else {
            leftSpeed = 1;
          }
        } else {
          leftSpeed = driveSpeed + angleCorrection;
        }
        
   

        if (Math.abs(driveSpeed - angleCorrection) > 1){
          if(driveSpeed - angleCorrection < 0) {
            rightSpeed = -1;
          } else {
            rightSpeed = 1;
          }
        } else {
          rightSpeed = driveSpeed - angleCorrection;
        }
        rightSpeed *= kAutoRightSpeedCorrection;
        SmartDashboard.putNumber("LeftSpeed", leftSpeed);
        SmartDashboard.putNumber("RightSpeed", rightSpeed);
    
        // Run the drive
        myDrivetrain.autoDrive(leftSpeed, rightSpeed);
        break;

    }

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("BallCount", ballCount);
    SmartDashboard.putNumber("RobotMode", robotMode);
    // Initialize finished flag
    boolean thereYet = false;


    // Get current time
    double time = runTimer.get();


    // Check for max time
    if (stopTime <= time - startTime) {

      // Set flag
      thereYet = true;

    } 
    else
    {
    
      // Determine mode and make checks
      switch(robotMode) {

        // Shooting
        case 1:

          // Check number of balls shot
          if (ballCount == 0) {

            if (!timeSet) {

              allShotsTime = runTimer.get();
              timeSet = true;

            } 
            else if (runTimer.get() - allShotsTime > shotWaitTime) {

              // Reset flag
              timeSet = false;

              // Set next mode
              robotMode = 2;

              // Reset starting encoder positions
              leftEncoderStart = myDrivetrain.getMasterLeftEncoderPosition();
              rightEncoderStart = myDrivetrain.getMasterRightEncoderPosition();
              pidDriveAngle = new PIDControl(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);
              driveDistance = 0;
              // Turn off shooter speed control
              runSpeedControl = false;
              turretLocked = false;

            }

          }

          break;
      
        // Driving to load area
        case 2:

          myProcessor.stopProcessor();
          // Calculate distance traveled
          totalRotationsRight = Math.abs(myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart);
          totalRotationsLeft = Math.abs(myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart);
          driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
          SmartDashboard.putNumber("Distance Traveled", driveDistance);

          // Check distance against target
          SmartDashboard.putNumber("Distance error", Math.abs(driveDistance - targetDriveDistance));
          if (driveDistance >= targetDriveDistance) {
            
            // Stop the robot
            myDrivetrain.stopDrive();

            // Set next mode
            robotMode = 3;

          }

          break;
      
        // Waiting in load area
        case 3:

          // Check light sensor for ball loading
          // if (!ballEntering) {
          //   if(myDrivetrain.getProcessorEntry() == false) {
          //     ballPresentCount++;
          //     if (ballPresentCount == 5){
          //       ballPresentCount = 0;
          //       ballEntering = true;
          //     }
          //   }       
          // }
          // else {
          //   if(myDrivetrain.getProcessorEntry() == true){
          //     ballCount++;
          //     ballEntering = false;
          //   }
          //}
          if (myDrivetrain.getProcessorEntry() == false){
            ballPresentCount++;
            if (ballPresentCount == 5){
              ballCount++;
              ballPresentCount = 0;
            }
          }

          // Check number of balls loaded
          if (ballCount == 3) {
            
            // Set next mode
            robotMode = 4;

            // Reset starting encoder positions
            leftEncoderStart = myDrivetrain.getMasterLeftEncoderPosition();
            rightEncoderStart = myDrivetrain.getMasterRightEncoderPosition();
            driveDistance = 0;
            pidDriveAngle = new PIDControl(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);
            // Turn on shooter speed control
            runSpeedControl = true;

          }
          
          break;
      
        // Driving to shooting location
        case 4:

          // Stop the intake processor
          myProcessor.stopProcessor();

          // Calculate distance traveled
          totalRotationsRight = Math.abs(myDrivetrain.getMasterRightEncoderPosition() - rightEncoderStart);
          totalRotationsLeft = Math.abs(myDrivetrain.getMasterLeftEncoderPosition() - leftEncoderStart);
          driveDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
          SmartDashboard.putNumber("Distance Traveled", driveDistance);

          // Check distance against target
          if (driveDistance >= (returnDriveDistance)) {
            
            // Stop the robot
            myDrivetrain.stopDrive();

            // Set next mode
            robotMode = 1;

          }

          break;
          
      }

    }


    // Return finished flag
    return thereYet;

  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myDrivetrain.stopDrive();
    myProcessor.stopProcessor();
    myShooter.stopShooter();
    myTurret.stopHood();
    myTurret.stopTurret();
    
  }

}
