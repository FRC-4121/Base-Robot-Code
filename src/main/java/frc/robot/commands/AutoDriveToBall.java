/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.extraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.extraClasses.NetworkTableQuerier;

public class AutoDriveToBall extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final Pneumatics shifter;
  private final NetworkTableQuerier ntables;
  // private final Processor processor;

  private double targetDistance;
  private double targetAngle;
  private double direction;
  private double stopTime;

  private double angleCorrection, angleError, speedCorrection;
  private double startTime;
  private double distanceTraveled;

  private double targetGyro;
  private double actualGyro;
  private boolean holdGyro;

  private double leftEncoderStart;
  private double rightEncoderStart;

  private boolean ballOnBoard;

  private Timer timer = new Timer();
  private PIDControl pidAngle; 


  public AutoDriveToBall(Drivetrain drive, Pneumatics shift, NetworkTableQuerier table, double time) {
    // processor = process;
    drivetrain = drive;
    shifter = shift;
    ntables = table;
    addRequirements(drivetrain, shifter);

    stopTime = time;

    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
    startTime = timer.get();

    angleCorrection = 0;
    angleError = 0;
    speedCorrection = 1;

    holdGyro = false;
    targetGyro = drivetrain.getGyroAngle();
    actualGyro = drivetrain.getGyroAngle();

    ballOnBoard = true;

    // shifter.shiftUp();
    // shifter.retractIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get vision values
    double ballOffset = ntables.getVisionDouble("BallOffset0");
    double ballDistance = ntables.getVisionDouble("BallDistance0");
    boolean foundBall = ntables.getVisionBoolean("FoundBall");
    SmartDashboard.putBoolean("FoundBAll", foundBall);
    

    // Read current gyro angle
    actualGyro = drivetrain.getGyroAngle();

    // Update target gyro angle unless we are holding
    if (holdGyro == false){
      targetGyro = actualGyro;
    }
    
    SmartDashboard.putNumber("ActualGyro", actualGyro);
    SmartDashboard.putNumber("TargetGyro", targetGyro);

    // Calculate correction based on ball offset if far away
    // or based on gyro angle if close enough
    if (holdGyro == false){
      angleCorrection = pidAngle.run(ballOffset, 0);
    } else {
      angleCorrection = pidAngle.run(actualGyro, targetGyro);
    }
    
    // Calculate speed correction based on distance
    if ((ballDistance > 30) && foundBall == true){
      speedCorrection = 1;
    }else{
      speedCorrection = 1;
    }
    SmartDashboard.putNumber("SpeedCorrect", speedCorrection);

    // Run drive train
    direction = -1;
    if (foundBall) {
      drivetrain.autoDrive(speedCorrection * direction * kAutoDriveSpeed + angleCorrection, speedCorrection * direction*kAutoDriveSpeed - angleCorrection);
    } else {
      drivetrain.stopDrive();//modified for showcase purposes
    }
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    // processor.runProcessor(false);
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

    ballOnBoard = drivetrain.getProcessorEntry();
 
    SmartDashboard.putNumber("Auto Time", time);
    SmartDashboard.putNumber("Auto Start Time", startTime);
    SmartDashboard.putBoolean("BallOnBoard", ballOnBoard);

    if(ballOnBoard == false) {
      thereYet = true;
    }
    else if (stopTime <= time - startTime){
      thereYet = true;
    }
    SmartDashboard.putBoolean("Auto TY", thereYet);
    return thereYet;

  }
}