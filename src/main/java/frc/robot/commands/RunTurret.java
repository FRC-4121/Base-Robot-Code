/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;;

public class RunTurret extends CommandBase {
 
  private Turret myTurret;
  private double speed;


  public RunTurret(Turret turret, double turretSpeed) {

    myTurret = turret;
    speed = turretSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myTurret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myTurret.rotateTurret(speed);
    
    // double turretAngle = myTurret.getTurretAngle();
    // if(speed > 0){
    //   if(turretAngle <= kTurretMinAngle){
    //     myTurret.rotateTurret(speed);
    //   }
    //   else if(turretAngle < kTurretMaxAngle)
    //   {
    //     myTurret.rotateTurret(speed);
    //   }
    //   else
    //   {
    //     myTurret.stopTurret();
    //   }
    // }
    // else if (speed < 0)
    // {
    //   if(turretAngle >= kTurretMaxAngle){
    //     myTurret.rotateTurret(speed);
    //   }
    //   else if(turretAngle > kTurretMinAngle){
    //     myTurret.rotateTurret(speed);
    //   }
    //   else
    //   {
    //     myTurret.stopTurret();
    //   }
    // }
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
