/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // 30AMP CURRENT LIMITS
    public static final int current30AmpPeakCurrentLimit = 25;
    public static final int current30AmpPeakCurrentDuration = 200;
    public static final int current30AmpContinuousCurrentLimit = 25;

    // 40AMP CURRENT LIMITS
    public static final int current40AmpPeakCurrentLimit = 35;
    public static final int current40AmpPeakCurrentDuration = 200;
    public static final int current40AmpContinuousCurrentLimit = 35;

     /*
    XBOX BUTTON MAP KEY:
    1 = A
    2 = B
    3 = X
    4 = Y
    5 = left bumper
    6 = right bumper
    7 = back
    8 = start
    9 = left stick click
    10 = right stick click
    */

    // Xbox Map
    public final int kXboxControllerPort = 0;

    public final int A = 1;
    public final int B = 2;
    public final int X = 3;
    public final int Y = 4;

    public final int LB = 5;
    public final int RB = 6;

    public final int BACK = 7;
    public final int START = 8;

    public final int LT = 2;
    public final int RT = 3;

    public final int LS = 9;
    public final int RS = 10;

    public final int LY = 1;
    public final int RX = 4;

    public final int LSX = 0;
    public final int LSY = 1;
    public final int RSX = 4;
    public final int RSY = 5;


 

    /**
     * 
     * Drive Constants
     * 
     */
    public static final class DriveConstants {

        // CAN
        public static final int leftMotorMasterID = 1;
        public static final int leftMotorSlaveID = 2;
        public static final int rightMotorMasterID = 3;
        public static final int rightMotorSlaveID = 4;
        public static final int indexMotorID = 5;

        public static final double wheelDiameter = 0.2032;
        public static final int pulsePerRevolution = 42;

        public static final boolean gyroReversed = false;

        public static final double turnToleranceDeg = 5.0;
        public static final double turnRateToleranceDegPerS = 10.0;

        public static final double moveKp = 0.1;
        public static final double moveKi = 0.0;
        public static final double moveKd = 0.0;

        public static final double turnKp = 0.085;
        public static final double turnKi = 0.003;
        public static final double turnKd = 0.5;

        public static final double kP = 5e-5;
        public static final double kI = 1e-6;
        public static final double kD = 0;

        public static final double kIz = 0; 
        public static final double kFF = 0.000156; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;

        //Gyro tuning paramenter indicates how close to 
        //On Target    
        public static final double kToleranceDegrees = 2.0f;

		public static final boolean LeftInvertedBoolean = false;
		public static final IdleMode IdleMode = null;
		public static final boolean RightInvertedBoolean = false;
        public static final double kDriveGearRatio = 10.71;
        public static final double kWheelDiameterMeters = 0.1524;

        public static final int kNeoBuiltinCPR = 42;
        //Convert the Rotations reported by built-in neo encoder to Meters as needed by the 
        // WPILIB's Ramsette controller
        public static final double kNeoPositionConversionFactor = (1/kDriveGearRatio) * Math.PI * kWheelDiameterMeters;
        // Convert RPM reported by built-in neo encoder to Meters/sec as needed by the 
        // WPILIB's Ramsette controller
        public static final double kNeoVelocityConversionFactor = (1/60) * (1/kDriveGearRatio) * Math.PI * kWheelDiameterMeters;
        // public static final int kEncoderCPR = kNeoBuiltinCPR;
        // public static final double kWheelDiameterInches = 6;
        // public static final double kEncoderDistancePerPulse =
        //     // Assumes the encoders are NOT directly mounted on the wheel shafts
        //     (kWheelDiameterInches * Math.PI) / (double) (kEncoderCPR * kDriveGearRatio);
        
        public static final double kTrackwidthMeters = 0.6713;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;


        public static final double kLeftEncoderPulsesPerRev = kNeoBuiltinCPR * 4 * kDriveGearRatio;
        public static final double kRightEncoderPulsesPerRev = kNeoBuiltinCPR * 4 * kDriveGearRatio;
        // public static final double kLeftEncoderPulsesPerRev = 8192; // Rev Throughbore encoder
        // public static final double kRightEncoderPulsesPerRev = 8192; // Rev Throughbore encoder
        public static final double kLeftMetersPerPulse = Math.PI * kWheelDiameterMeters / kLeftEncoderPulsesPerRev;
        public static final double kRightMetersPerPulse = Math.PI * kWheelDiameterMeters / kRightEncoderPulsesPerRev;


        public static final int kNeoEncoderPulsesPerRev = kNeoBuiltinCPR * 4;
        public static final double kNeoEncoderMetersPerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) (kNeoEncoderPulsesPerRev * kDriveGearRatio);
    
        public static final boolean kGyroReversed = true;
    
        // kS, kV, kA values from frc-characterization tool
        public static final double ksVolts = 0.15;
        public static final double kvVoltSecondsPerMeter = 2.71;
        public static final double kaVoltSecondsSquaredPerMeter = 0.367;
        // public static final double kTrackwidthMeters = 0.6713;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.34; // value from characterization tool is 13.4!
       


        
        



        /**
         * 
         * Shooter Constants
         * 
         */
        public static final class ShooterConstants {

            // CAN
            public static final int topMotorID = 6;
            public static final int bottomMotorID = 7;

            public static final double speed = 0.5;
            public static final double backSpeed = -0.4;
            public static final double unitsPerRotation = 4096.0;
            public static final double tolerance = 200.0;
            public static final int kPIDLoopIdx = 0;
            public static final int kTimeoutMs = 30;
            public static final double targetHeight = 99.0;

            // this was change in angle / change in height of cam from ground
            public static final double angleHeightMultiplier = 0.294;

            public static final double closestRangeInches = 120.0;
            public static final double farthestRangeInches = 300.0;

            public static final double closestRangeTopRPM = 1000;
            public static final double farthestRangeTopRPM = 2000;

            public static final double closestRangeBottomRPM = 2900;
            public static final double farthestRangeBottomRPM = 4400;

            public static final double RPMLowLimit = 700;
            public static final double RPMHighLimit = 4800;

            //PID
            public static final double kP =  6e-5;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kIz = 0;
            public static final double kFF = 0.000015;
            public static final double kMaxOutput = 1;
            public static final double kMinOutput = -1;
            

        }

        /**
         * 
         * Spinner Constants
         * 
         */
        public static final class SpinnerConstants {

            // CAN
            public static final int motorID = 9;

            public static final double speed = 0.6;

            // mathing
            public static final int unitsPerRotation = 4096;
            public final static double wheelDiameter = 2.0;
            public final static double targetSpins = 3.5;
            public final static double targetUnitsForTargetSpins = ((100 / (wheelDiameter * Math.PI)) * targetSpins)
                    * unitsPerRotation;

        }

    }

    /**
         * Indexer Constants
         */
        public static final class IndexerConstants {
            // CAN
            public static final int indexMotorID = 8;
            public static final double speed = 0.5;
            public static final double backSpeed = -0.4;
            public static final double unitsPerRotation = 42.0;
            public static final double tolerance = 200.0;
            public static final int kPIDLoopIdx = 0;
            public static final int kTimeoutMs = 30;

        }

        public static final class OIConstants {
            public static final int kDriverControllerPort = 0;
            public static int kOtherControllerPort = 1;
        }
    
        public static final class AutoConstants {
            public static final double kMaxSpeedMetersPerSecond = 3;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
            // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;
        }

        //AutoPaths must be generated
    
        public static final class AutoPathsConstants {
            public static final int kPos3Path1_numSegments = 4;
            public static final String[] kPos3Path1 = 
                new String[] {"paths/Auto_pos3_path1_segment1.wpilib.json", "paths/Auto_pos3_path1_segment2.wpilib.json",
                    "paths/Auto_pos3_path1_segment3.wpilib.json", "paths/Auto_pos3_path1_segment4.wpilib.json"};
        }

	

}
