// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ProcessorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Processor2 extends SubsystemBase {
  /** Creates a new Processor2. */
  private WPI_TalonSRX intake = new WPI_TalonSRX(INTAKE);
  private CANSparkMax processorMain = new CANSparkMax(PROCESSOR_MAIN, MotorType.kBrushless);
  private WPI_TalonSRX lock = new WPI_TalonSRX(PROCESSOR_END);
 
  public Processor2() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runProcessor(boolean invertDirection){

    if(!invertDirection)
    {
      processorMain.set(kProcessorSpeed);
      intake.set(kIntakeSpeed);
    }
    else 
    {
      processorMain.set(-kProcessorSpeed);
      intake.set(kOuttakeSpeed);
    }
    lock.set(kLockSpeed);
  }

  public void autoRunProcessor(boolean invertDirection, boolean useBar){

    if(!invertDirection)
    {
      if (useBar) intake.set(kIntakeSpeed);
      processorMain.set(kProcessorSpeed);
      lock.set(kLockSpeed);
      // lockProcessor();
      
    }
    else 
    {
      if (useBar) intake.set(kIntakeSpeed);
      processorMain.set(-kProcessorSpeed);
    }
  }

  public void unlockProcessor(){

    lock.set(kUnlockSpeed);
  }

  public void lockProcessor(){

    lock.set(kLockSpeed);
  }

  public void stopProcessor(){

    intake.set(0);
    processorMain.set(0);
    lock.set(0);
  }

}
