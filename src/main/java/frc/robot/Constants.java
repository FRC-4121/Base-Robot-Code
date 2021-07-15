/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

    //Spark Max IDs
    //With NEOs
    public static final int PROCESSOR_MAIN = 8;
    public static final int TURRET = 9;
    public static final int HOOD = 3;

    //Talon SRX and FX IDs (must be unique, may range from 0+)
    // public static final int LEFT_MASTER_F = 3;
    // public static final int LEFT_SLAVE_F = 5;
    // public static final int RIGHT_MASTER_F = 2;
    // public static final int RIGHT_SLAVE_F = 4;
    // public static final int INTAKE = 7;
    // public static final int PROCESSOR_END = 6;
    // public static final int SHOOTER_MASTER = 0;
    // public static final int SHOOTER_SLAVE = 1;

    public static final int LEFT_MASTER_F = 3;
    public static final int LEFT_SLAVE_F = 5;
    public static final int RIGHT_MASTER_F = 2;
    public static final int RIGHT_SLAVE_F = 4;
    public static final int INTAKE = 7;
    public static final int PROCESSOR_END = 6;
    public static final int SHOOTER_MASTER = 1;
    public static final int SHOOTER_SLAVE = 0;

    //Drive control port IDs
    public static final int XBOX_PORT = 0;
    public static final int LEFT_JOYSTICK = 1;
    public static final int RIGHT_JOYSTICK = 2;

    //DigitalInput port IDs *move these to their respective subclasses please*
    public static final int TURRET_LIMIT_SWITCH  = 2;
    public static final int PROCESSOR_INDEX_1 = 0;
    public static final int PROCESSOR_INDEX_2 = 4;



    public static class DrivetrainConstants {

        public static final boolean kMotorInvert = true;//True -> right side motors are inverted
        public static final int kPIDLoopIdxDrive = 0;
        public static final int kTimeoutMsDrive = 20;
        public static final double kTalonFXPPR = 2048;
        public static final double kWheelDiameter = 6.0;
        public static final double kLowGearSpeedCap = 0.8;//In case full speed draws excessive power, these are an emergency measure
        public static final double kHighGearSpeedCap = 1.0;
        public static double kJoystickSpeedCorr = 0.55;
        public static final double kAutoDriveSpeed = .4;
        public static final double kAutoDriveSpeedMin = 0.25;
        public static final double kAutoShootDriveSpeed = 0.75;
        public static final double kAutoTurnSpeed = 0.5;
        // public static final double kLowGearRatio = 30.0;
        // public static final double kHighGearRatio = 70.0;
        public static final double kGearRatio = 7; 
        public static final double kTurnAngleTolerance = 0.001;
        public static final double kDriveDistanceTolerance = 10.0;
        public static final double AUTO_ENCODER_REVOLUTION_FACTOR = 14750.0;

        public static final double kP_Straight = 0.012;  //was 0.024
        public static final double kI_Straight = 0.0;
        public static final double kD_Straight = 0.0;
        public static final double kP_Turn = .002;//was .003
        public static final double kI_Turn = 0.0;
        public static final double kD_Turn = 0.001;//was 0.0004
        public static final double kP_DriveAngle = .003;//was .005
        public static final double kI_DriveAngle = 0.0;
        public static final double kD_DriveAngle = 0.0004;

        public static final double kAutoRightSpeedCorrection = 0.9625;

        //Filtering (for gyro)
        public static final int FILTER_WINDOW_SIZE = 150;

        public static int DIRECTION_MULTIPLIER = 1;//Controls whether forward on joysticks is forward or backward on robot
    }

    public static class ShooterConstants {

        //PID constants
        public static final double kP_Shoot = 0.00015; //was 0.1
        public static final double kI_Shoot = 0.0000;
        public static final double kD_Shoot = 0;
        public static final double kF_Shoot = -1; 

        public static final double kP_Turret = 0.01;
        public static final double kI_Turret = 0.0000;
        public static final double kD_Turret = 0.000;

        public static final double kP_TurretLock = 0.00115;
        public static final double kI_TurretLock = 0.0005;//try .0006
        public static final double kD_TurretLock = 0.000055;//try 0.000007

        public static final double kTurretAngleTolerance = 2.0;

        public static final double kSpeedCorrectionFactor = 1.08;
        public static final double kRPMTolerance = 100;

        public static final int kPIDLoopIdxShoot = 0;
        public static final int kTimeoutMsShoot = 20;
        public static final int kShooterMaxRPM = 6100;

        public static final double kTalonFXPPR = 2048;
        public static final double kTurretEncoderPPR = 10.5;

        //Turret dimensions/other config
        public static final double kTurretGearReduction = .5;
        public static final double kTurretSprocketRatio = 16.0/120;
        public static final double kTurretDiskDia = 13.75;

        public static final double kTurretMinAngle = -30;
        public static final double kTurretMaxAngle = 30;
        //0 is centered

        public static final int kHoodEncoder = 0;
        public static final double kHoodSpeed = -1.0;

        //Speeds
        public static final double kShooterSpeed = -1.0;
        public static final double kTurretSpeedManual = 0.10;//positive is counterclockwise
        public static final double kTurretSpeedAuto = 1.0;
        public static final double kTurretSpeedLock = 0.0625;
    }

    public static class PneumaticsConstants {

        public static final int[] SHIFTER = {1, 6};
        public static final int[] INTAKE_PNEU = {0, 7};

        public static String GEAR = "Low";
        public static String INTAKE_STATUS = "Retracted";
        public static String PTO_STATUS = "Disengaged";
        public static String KICKSTAND_STATUS = "Up";
    }

    public static class ProcessorConstants {

        //All values experimental
        public static final double kIntakeSpeed = -.55;
        public static final double kOuttakeSpeed = .75;
        public static final double kProcessorSpeed = -0.3;
        public static final double kUnlockSpeed = .8;
        public static final double kLockSpeed = -.20;
    }

}
