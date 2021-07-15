/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.*;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.extraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final Pneumatics shifter;

  private double targetDriveDistance;
  private double targetAngle;
  private double direction;
  private double stopTime;
  private double currentGyroAngle = 0;
  private double driveDirection;
  private double driveSpeed;

  private double angleCorrection, speedCorrection;
  private double startTime;
  private double distanceTraveled;

  private double leftEncoderStart;
  private double rightEncoderStart;
  private double totalRotationsLeft = 0;
  private double totalRotationsRight = 0;


  private Timer timer = new Timer();
  private PIDControl pidDriveAngle;
  private PIDControl pidDriveDistance; 


  public AutoDrive(Drivetrain drive, Pneumatics shift, double dis, double ang, double dir, double time) {

    drivetrain = drive;
    shifter = shift;
    addRequirements(drivetrain, shifter);

    targetDriveDistance = dis;
    targetAngle = ang;
    direction = dir;
    stopTime = time;

    pidDriveAngle = new PIDControl(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);
    pidDriveDistance = new PIDControl(kP_Straight, kI_Straight, kD_Straight);
    //pidSpeedHigh = new PIDControl(kP_Speed_High, kI_Speed_High, kD_Speed_High);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    distanceTraveled = 0.0;
    timer.start();
    startTime = timer.get();

    leftEncoderStart = drivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = drivetrain.getMasterRightEncoderPosition();

    angleCorrection = 0;
    speedCorrection = 1;

    pidDriveAngle = new PIDControl(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);

    shifter.shiftUp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calculate angle correction based on gyro reading
    currentGyroAngle = drivetrain.getGyroAngle();
    angleCorrection = pidDriveAngle.run(currentGyroAngle, 0);
    // SmartDashboard.putNumber("AngleCor", angleCorrection);

    // Calculate speed correction based on distance to target
    totalRotationsRight = Math.abs((Math.abs(drivetrain.getMasterRightEncoderPosition()) - rightEncoderStart));
    totalRotationsLeft = Math.abs((Math.abs(drivetrain.getMasterLeftEncoderPosition()) - leftEncoderStart));
    distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
    // speedCorrection = pidDriveDistance.run(distanceTraveled, targetDriveDistance);
    // SmartDashboard.putNumber("DrSpCorrection", speedCorrection);

    //Check for overspeed correction
    // if(speedCorrection > 1){

    //   speedCorrection = 1;

    // } else if (speedCorrection < -1) {

    //   speedCorrection = -1;

    // }

    // Calculate final drive speed
    speedCorrection = 1;
    // if (Math.abs(distanceTraveled - targetDriveDistance) <= 18) {
    //   // speedCorrection = 0.5;
    // }
    // direction = 1;
    driveSpeed = direction * speedCorrection * kAutoShootDriveSpeed;

    SmartDashboard.putNumber("DriveSpeed", driveSpeed);
    // Enforce minimum speed
    // if (Math.abs(driveSpeed) < kAutoDriveSpeedMin) {

    //   angleCorrection = 0;
    //   if (driveSpeed < 0){
    //     driveSpeed = -kAutoDriveSpeedMin;
    //   } else {
    //     driveSpeed = kAutoDriveSpeedMin;
    //   }
    // }
    SmartDashboard.putNumber("AngleCor", angleCorrection);

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
    drivetrain.autoDrive(leftSpeed, rightSpeed);

    double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
    double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));

    distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;

    double time = timer.get();
    totalRotationsRight = Math.abs((Math.abs(drivetrain.getMasterRightEncoderPosition()) - rightEncoderStart));
    totalRotationsLeft = Math.abs((Math.abs(drivetrain.getMasterLeftEncoderPosition()) - leftEncoderStart));
    distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
    SmartDashboard.putNumber("DistanceTraveled", distanceTraveled);

    // Check distance against target
    SmartDashboard.putNumber("Distance error", Math.abs(distanceTraveled - targetDriveDistance));
    if (distanceTraveled >= targetDriveDistance) {
      drivetrain.stopDrive();
      if (time - startTime >= stopTime){
        thereYet = true;
      }
    }

    return thereYet;

  }
}
