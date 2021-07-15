/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor2;

public class AutoShoot extends CommandBase {
  /**
   * Creates a new AutoShoot.
   */
  Processor2 processor;
  
  
  private Timer timer = new Timer();
  double shotTime;
  double startTime;
  double timeBetweenShot;
  double timeForShot = .2;

  double shotCount;
  double shotsToTake;
  boolean takeShot = false;

  public AutoShoot(Processor2 process, double time, double shots) {
    processor = process;
    
    timeBetweenShot = time;
    shotsToTake = shots;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(processor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    startTime = timer.get();
    shotCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(shotCount == 0){
      
      takeShot = true;
    } else if (timer.get() - shotTime >= timeBetweenShot){

      takeShot = true;

    } else {
      takeShot = false;
    }
    
    if (takeShot){
      processor.unlockProcessor();  
      
      shotTime = timer.get();
    } else if (timer.get() - shotTime >= timeForShot)
    {
      shotCount++;
      processor.lockProcessor();
    }

    processor.runProcessor(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean thereYet = false;
    
    if(shotCount >= shotsToTake)
    {
      thereYet = true;
      processor.lockProcessor();
      processor.stopProcessor();
    }

    return thereYet;
  }
}
