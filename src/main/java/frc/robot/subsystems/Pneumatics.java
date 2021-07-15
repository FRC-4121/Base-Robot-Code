/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticsConstants.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  
  private Compressor compressor = new Compressor(0);

  private DoubleSolenoid shifter = new DoubleSolenoid(SHIFTER[0], SHIFTER[1]);
  private DoubleSolenoid intakePneu = new DoubleSolenoid(INTAKE_PNEU[0], INTAKE_PNEU[1]);

  public Pneumatics() {

    //Startup configurations
    //retractIntake();
    //disengagePTO();
    //shiftDown();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Drive Gear", GEAR);
    SmartDashboard.putString("Intake Status", INTAKE_STATUS);
  }

  public void extendIntake() {
    
    intakePneu.set(Value.kForward);//will require testing of solenoid to confirm
    INTAKE_STATUS = "Extended";
  }

  public void retractIntake() {

    intakePneu.set(Value.kReverse);//will require testing of solenoid to confirm
    INTAKE_STATUS = "Retracted";
  }

  public void shiftDown() {

    shifter.set(Value.kReverse);//will require testing
    GEAR = "Low";
  }

  public void shiftUp() {

    shifter.set(Value.kForward);
    GEAR = "High";
  }
}
