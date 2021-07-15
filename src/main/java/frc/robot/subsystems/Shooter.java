/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  // Initialize motor controllers
  private final TalonFX shooterMaster = new WPI_TalonFX(SHOOTER_MASTER);
  // private final TalonFX shooterSlave = new WPI_TalonFX(SHOOTER_SLAVE);
  
  // //for testing
  private double save_p = kP_Shoot;
  private double save_i = kI_Shoot;
  private double save_d = kD_Shoot;
  private double save_f = kF_Shoot;
  private double speed = 0;
 

  public Shooter() {

    //Configure slave (Falcons currently unsupported)
    //shooterSlave.follow(shooterMaster);
    //shooterSlave.setInverted(InvertType.OpposeMaster);

    //Set up encoder for speed control
    shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxShoot, kTimeoutMsShoot);
    // shooterSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxShoot, kTimeoutMsShoot);
    
    //Config speed PID
		shooterMaster.config_kP(kPIDLoopIdxShoot, kP_Shoot, kTimeoutMsShoot);
		shooterMaster.config_kI(kPIDLoopIdxShoot, kI_Shoot, kTimeoutMsShoot);
    shooterMaster.config_kD(kPIDLoopIdxShoot, kD_Shoot, kTimeoutMsShoot);
    shooterMaster.config_kF(kPIDLoopIdxShoot, kF_Shoot, kTimeoutMsShoot);

    // shooterSlave.config_kP(kPIDLoopIdxShoot, kP_Shoot, kTimeoutMsShoot);
		// shooterSlave.config_kI(kPIDLoopIdxShoot, kI_Shoot, kTimeoutMsShoot);
    // shooterSlave.config_kD(kPIDLoopIdxShoot, kD_Shoot, kTimeoutMsShoot);
    // shooterSlave.config_kF(kPIDLoopIdxShoot, kF_Shoot, kTimeoutMsShoot);
    
    //for testing, we put PID tuning in the smart dash
    SmartDashboard.putNumber("P Shoot", kP_Shoot);
    SmartDashboard.putNumber("I Shoot", kI_Shoot);
    SmartDashboard.putNumber("D Shoot", kD_Shoot);
    SmartDashboard.putNumber("F Shoot", kF_Shoot);
    SmartDashboard.putNumber("Speed", speed);

  }


  /**
   * Actions to take every processor cycle
   */
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
    
    // // For testing we grab the PID from the smart dash
    // double p = SmartDashboard.getNumber("P Shoot", save_p);
    // double i = SmartDashboard.getNumber("I Shoot", save_i);
    // double d = SmartDashboard.getNumber("D Shoot", save_d);
    // double f = SmartDashboard.getNumber("F Shoot", save_f);
    // speed = SmartDashboard.getNumber("Speed", kShooterSpeed);
    
    // if(p != save_p) {
    //   shooterMaster.config_kP(kPIDLoopIdxShoot, p, kTimeoutMsShoot);
    //   // shooterSlave.config_kP(kPIDLoopIdxShoot, p, kTimeoutMsShoot);
    //   save_p = p;
    // }
    // if(i != save_i) {
    //   shooterMaster.config_kI(kPIDLoopIdxShoot, i, kTimeoutMsShoot);
    //   // shooterSlave.config_kI(kPIDLoopIdxShoot, i, kTimeoutMsShoot);
    //   save_i = i;
    // }
    // if(d != save_d) {
    //   shooterMaster.config_kD(kPIDLoopIdxShoot, d, kTimeoutMsShoot);
    //   // shooterSlave.config_kD(kPIDLoopIdxShoot, d, kTimeoutMsShoot);
    //   save_d = d;
    // }
    // if(f != save_f) {
    //   shooterMaster.config_kF(kPIDLoopIdxShoot, f, kTimeoutMsShoot);
    //   // shooterSlave.config_kF(kPIDLoopIdxShoot, f, kTimeoutMsShoot);
    //   save_f = f;
    // }

    // //Warning - this will run all the time!!!
    // shooterMaster.set(ControlMode.Velocity, speed * 1023);
    // shooterSlave.set(ControlMode.Velocity, -speed * 1023);

  }


  /**
   * Run the shooter at a percent speed
   * @param speed1
   */
  public void shoot(double speed1){

    speed = speed1; 
    shooterMaster.set(ControlMode.PercentOutput, speed);
    // shooterSlave.set(ControlMode.PercentOutput, -speed);
    
  }

  /**
   * Run the shooter at a RPM
   * @param rpm: The desired RPM for the flywheel
   */
  public void shootRPM(double rpm){
    //Speed for velocity control is encoder units per 100ms (why, I honestly couldn't tell you)
    speed = rpm * kTalonFXPPR / (60 * 10);
    shooterMaster.set(ControlMode.Velocity, speed);
  }

  /**
   * Stop the shooter
   */
  public void stopShooter(){

    shooterMaster.set(ControlMode.PercentOutput, 0);
    // shooterSlave.set(ControlMode.PercentOutput, 0);
  }


  /**
   * Get the RPM of the shooter wheel
   * @return
   */
  public double getShooterRPM(){

    return shooterMaster.getSelectedSensorVelocity() * 10 * 60 / kTalonFXPPR;
  }


  /**
   * Get the speed of the shooter wheel
   * @return
   */
  public double getShooterSpeed(){

    return speed;
  }

  // public TalonFX getShooterMaster(){
  //   return shooterMaster;
  // }
  // public TalonFX getShooterSlave(){
  //   // return shooterSlave;
  // }

}
