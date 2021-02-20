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
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        public static final int kLeftDriverController = 0;
        public static final int kRightDriverController = 1;
        public static final int kAttachmentsController = 2;
        // If in single controller mode, put the controls on each button
        public static enum ControllerSetup {
            SINGLE_CONTROLLER,
            DIAGNOSTIC,
            COMPETITION
        }
    }

    public static final class AccessoryConstants {
        public static final int kGathererPort = 14;
        public static final int kMagazineMotorId = 11;
    }

    public static final class DriveConstants {
        public static final int kRightMotor1Port = 1;
        public static final int kRightMotor2Port = 3;
        public static final int kLeftMotor1Port = 2;
        public static final int kLeftMotor2Port = 4;
        public static final int kGathererMotor = 5;
        public static final int kFeederMotor = 6;

        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;

        public static final double kMaxSpeed = 4.0; // meters per second
        public static final double kMaxAngularSpeed = 3 * Math.PI; // one rotation per second
        public static final double kGathererSpeed = .5;
      
        public static final double kTrackWidth = 0.6597507168526806; // meters
        public static final double kWheelDiameter = 0.2032; // meters
        public static final double kDriveGearing = 10.75;
        
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 2.15;
        public static final double kaVoltSecondsSquaredPerMeter = 0.544;

        public static final double kLeftP = 14.5;
        public static final double kRightP = 14.5;

        public static final double kTurnP = .075;
        public static final double kTurnI = 0;
        public static final double kTurnD = .000001;
        public static final boolean kGyroReversed = true;

        public static final double kTurnToleranceDeg = 10;
        public static final double kTurnRateToleranceDegPerS = 10;//degree per sencond

        public static final int kCurrentLimit = 40;

        public static final double kSlowFactor = 0.5;
        public static final double kDeadBand = 0.05;
    }

    public static final class ShooterConstants {
        public static final int kShooterId = 20;
        public static final boolean kShooterReversed = false; // Reversed as of late 2/27/2020

        public static final int  kServoActuatorId = 0;

        public static final double kMaxPercentOutput = 0.65;
        public static final double kF = 0.1968269231;
        public static final double kP = 0.1;
        public static final double kEncoderConstant = 1.462847143; // multiply desired rpm by this to get encoder units

        public static final double kDesiredAngularSpeed = 6000.00; // encoder units
        public static final double kDesiredActiveCurrent = 24.00; // in amps

        public static final double kFeedThresholdAngularSpeedDelta = 25.00;

        public static final double kShootThresholdAngularSpeedDelta = -50; // placeholder value
        public static final double kShootThresholdActiveCurrentDelta = 4.000;

        public static final double kDelay = 1.0;

        public static final double kShootThreeBallsTime = 6.0;

        public static final double kContinuousShootSpinUpTime = 2.0;
    }

    public static final class MagazineConstants {
        public static final int kInSensorPort = 0; // Analog input port
        public static final int kOutSensorPort = 1; // Analog input port
        public static final double kMaxSpeed = 0.35;
        public static final double kSensorThreshold = 300;
        public static final double kDelay = 0.2;
    }

    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class LEDConstants{
        public static final int kLEDPwmPort = 9;
        public static final int kLEDStrandLength = 150;
    }

    public static final class ClimberConstants{
        public static final int kWinchMotor = 7;
        public static final int kCurrentLimit = 40;
        public static final double kWinchSpeed = 1;
    }
}
