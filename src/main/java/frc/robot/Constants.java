/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

    // Xbox-Controller
    public static final int portXboxController = 0;

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
        public static final int pulsePerRevolution = 360;

        public static final boolean gyroReversed = false;

        public static final double turnToleranceDeg = 5.0;
        public static final double turnRateToleranceDegPerS = 10.0;

        public static final double moveKp = 0.0;
        public static final double moveKi = 0.0;
        public static final double moveKd = 0.0;

        public static final double turnKp = 0.085;
        public static final double turnKi = 0.003;
        public static final double turnKd = 0.5;

        /**
         * Indexer Constants
         */
        public static final class IndexerConstants {
            // CAN
            public static final int indexMotorID = 8;
            public static final double speed = 0.5;
            public static final double backSpeed = -0.4;
            public static final double unitsPerRotation = 4096.0;
            public static final double tolerance = 200.0;
            public static final int kPIDLoopIdx = 0;
            public static final int kTimeoutMs = 30;

        }

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

}
