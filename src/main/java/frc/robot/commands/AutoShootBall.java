// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootBall extends CommandBase {
 
  private final Shooter shooter;
  private double stopTime;
  private double startTime;

  private Timer timer = new Timer();


  public AutoShootBall(Shooter shoot, double time) {
    
    shooter = shoot;
    addRequirements(shooter);

    stopTime = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    startTime = timer.get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.shoot(-0.8);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean thereYet = false;

    double time = timer.get();
 
    if (stopTime <= time - startTime){

      thereYet = true;
    }
  
    return thereYet;

  }
}
