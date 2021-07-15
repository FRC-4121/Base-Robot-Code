/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Turret extends SubsystemBase {

  //Declare turret motor and encoder
  private final CANSparkMax turret = new CANSparkMax(TURRET, MotorType.kBrushless);
  private final CANEncoder turretEncoder = turret.getEncoder();
  // private final WPI_TalonFX turret = new WPI_TalonFX(TURRET);

  private final CANSparkMax hood = new CANSparkMax(HOOD, MotorType.kBrushless);
  private final AnalogInput hoodEncoder = new AnalogInput(kHoodEncoder);

  //Declare class variables
  private boolean resetEncoder = false;
  private double turretAngle;


  /**
   * Creates a new Turret.
   */
  public Turret() {

    //Configured to calculate the angle around the turret from the limit switch
    //theta (radians) = arclength / radius
    // turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxShoot, kTimeoutMsShoot);
    // turret.setSelectedSensorPosition(0);
    // turret.setNeutralMode(NeutralMode.Brake);
    turretEncoder.setPosition(0);
    // turretEncoder.setPositionConversionFactor(-kTurretSprocketDia * 360 / (kTurretEncoderPPR * kTurretGearReduction * kTurretDiskDia/2));
  }


  /**
   * Actions to take on a periodic basis
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // turretAngle = turret.getSelectedSensorPosition() * 360 * kTurretSprocketRatio / (kTalonFXPPR);
    turretAngle = turretEncoder.getPosition() * 360 * kTurretSprocketRatio / (kTurretEncoderPPR * kTurretGearReduction);
    SmartDashboard.putNumber("Turret Angle", turretAngle);
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
  }
  /**
   * Rotate the turret
   * @param speed
   */
  public void rotateTurret(double speed){
    SmartDashboard.putNumber("TurretSpeed", speed);
    double turretAngle = getTurretAngle();
    if(speed > 0){
      if(turretAngle <= kTurretMinAngle){
        turret.set(speed);
      }
      else if(turretAngle < kTurretMaxAngle)
      {
        turret.set(speed);
      }
      else
      {
        stopTurret();
      }
    }
    else if (speed < 0)
    {
      if(turretAngle >= kTurretMaxAngle){
        turret.set(speed);
      }
      else if(turretAngle > kTurretMinAngle){
        turret.set(speed);
      }
      else
      {
        stopTurret();
      }
    }
  }


  /**
   * Stop the turret
   */
  public void stopTurret(){

    turret.set(0);

  }

  public void rotateHood(double speed){
    hood.set(speed);
  }
  public void stopHood(){
    hood.set(0);
  }

  /**
   * Get the current angle of the turret
   * @return
   */
  public double getTurretAngle(){

    return turretAngle;
  }

  public double getHoodAngle(){
    return (hoodEncoder.getVoltage()/5.0)*360.0;
  }
}
