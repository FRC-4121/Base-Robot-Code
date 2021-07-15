// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.ShooterConstants.*;
public class RunHoodToPos extends CommandBase {
  /** Creates a new RunHoodToPos. */
  //Angle runs backwards on the encoder, so be aware of this!!!
  private Turret turret;
  private double targetAngle;
  public RunHoodToPos(Turret myTurret, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = myTurret;
    targetAngle = angle;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = turret.getHoodAngle();

    if (currentAngle < targetAngle){
      turret.rotateHood(kHoodSpeed);
    }
    else if (currentAngle > targetAngle){
      turret.rotateHood(-kHoodSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean thereYet = false;
    double currentAngle = turret.getHoodAngle();

    if (Math.abs(currentAngle - targetAngle) < 1){
      thereYet = true;
    }
    return thereYet;
  }
}
