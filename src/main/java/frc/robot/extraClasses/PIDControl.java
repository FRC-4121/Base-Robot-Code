/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.extraClasses;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDControl {

    private double kP, kI, kD;
    private double targetError, previousError;
    private double errorSum, errorChange;
    private double timeStep = 0.02;
    private double correctionFactor;

    public PIDControl(double gainP, double integralI, double derivativeD){

        kP = gainP;
        kI = integralI;
        kD = derivativeD;

        reset();
    }

    public double run(double sensorReading, double targetValue) {

        correctionFactor = 0;

        targetError = sensorReading - targetValue;

        //Calculate new correction
        errorSum += targetError * timeStep;
        errorChange = (targetError - previousError) / timeStep;
        correctionFactor = kP * targetError + kI * errorSum + kD * errorChange;
        SmartDashboard.putNumber("PContribute", kP * targetError);
        SmartDashboard.putNumber("IContribute", kI * errorSum);
        SmartDashboard.putNumber("DContribute", kD * errorChange);

        //Set previous error
        previousError = targetError;

        return correctionFactor;
    }

    public void reset() {

        errorSum = 0;
        errorChange = 0;
        targetError = 0;
        previousError = 0;
    }
}
