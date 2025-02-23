// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {

    public static final double odometryFrequency = 100.0; // Hz

    //  Should get overwritten in robotcontainer
    public static double maxSpeedMetersPerSec = 5.450; //4.0; // was default
     public static final double angularVelocityMultiplier = 0.5;
    public static final double trackWidth = Units.inchesToMeters(21);
    public static final double wheelBase = Units.inchesToMeters(21);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);


   // public static Translation2d[] moduleTranslations = null;
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };


    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0);

    // Device CAN IDs
    public static final int pigeonCanId = 9;

    public static final int frontLeftDriveCanId = 51;
    public static final int backLeftDriveCanId = 55;
    public static final int frontRightDriveCanId = 58;
    public static final int backRightDriveCanId = 62;

    public static final int frontLeftTurnCanId = 50;
    public static final int backLeftTurnCanId = 56;
    public static final int frontRightTurnCanId = 57;
    public static final int backRightTurnCanId = 49;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 40;
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveMotorReduction = 7.3; //(45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNEO(1); // DCMotor.getNeoVortex(1);

    //  Individual drive motor inversions
    public static final boolean frontLeftDriveMotorInverted = false;
    public static final boolean frontRightDriveMotorInverted = true;
    public static final boolean backLeftDriveMotorInverted = false;
    public static final boolean backRightDriveMotorInverted = true;

    //  Encoder data
    public static final int frontLeftDriveAbsoluteEncoderPort = 33;
    public static final double frontLeftDriveAbsoluteEncoderOffsetDeg = -.230225;
    public static final SensorDirectionValue frontLeftEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final int frontRightDriveAbsoluteEncoderPort = 30;
    public static final double frontRightDriveAbsoluteEncoderOffsetDeg = -0.261475;
    public static final SensorDirectionValue frontRightEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final int backLeftDriveAbsoluteEncoderPort = 32;
    public static final double backLeftDriveAbsoluteEncoderOffsetDeg = -0.394287;
    public static final SensorDirectionValue backLeftEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final int backRightDriveAbsoluteEncoderPort = 31;
    public static final double backRightDriveAbsoluteEncoderOffsetDeg = 0.397461;
    public static final SensorDirectionValue backRightEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.01;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.184445;  //  Got this value from characterization auto routine
    public static final double driveKv = 0.093025;  //  Got this value from characterization auto routine
    public static final double driveSimP = 0.01; //8.0;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.184445; //0.0;
    public static final double driveSimKv = 0.093025; //0.0789;

    // Turn motor configuration
    //  public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 21.0;  //9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1); //DCMotor.getNeo550(1);

    //  Individual turn motor inversions
    public static final boolean frontLeftTurnMotorInverted = true;
    public static final boolean frontRightTurnMotorInverted = true;
    public static final boolean backLeftTurnMotorInverted = true;
    public static final boolean backRightTurnMotorInverted = true;

    //  Individual turn motor inversions
    public static final boolean frontLeftTurnEncoderInverted = false;
    public static final boolean frontRightTurnEncoderInverted = false;
    public static final boolean backLeftTurnEncoderInverted = false;
    public static final boolean backRightTurnEncoderInverted = false;

    // Turn encoder configuration
    //  public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    //  Networktables stuff for tuning pid

    // Turn PID configuration
    public static final double turnKp = 0.4;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 0.4;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2.0 * Math.PI; // Radians


   // public static RobotConfig robotConfig;

   public static final double robotMassKg = 54.4311;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);

  }


    //  NOTICE: None of these values have been tested. They are all arbitrary
  public static final class ElevatorConstants {

    public static final double elevatorAbsoluteEncoderOffset = 0.0;
    public static final SensorDirectionValue elevatorEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double handoffHeight = 50;
    public static final double l2Height = 100;
    public static final double l3Height = 200;
    public static final double l4Height = 300;

    public static final double elevatorSoftLowerBound = 0;
    public static final double elevatorSoftUpperBound = 300;

    public static final int destinationTolerance = 10;
  }

  public static final class IntakeConstants {

    // NOTICE: None of these value have been tested as well.

    public static final double extendedPosition = 100.0;
    public static final double retractedPosition = 0.0;
    public static final double extensionTolerance = 5.0;
    public static final double climbPosition = 90.0;

    public static final double intakeSoftLowerBound = 0;
    public static final double intakeSoftUpperBound = 200;
  }

  public static final class ClimbConstants{
    // This values have not been tested.

    public static  final double climbOutPosition = 90;
    public static final double climbInPosition = 0;

    public static final double climbSoftLowerBound = 0;
    public static final double climbSoftUpperBound = 200;
  }
  public static final class AlgaeDislodgerConstants{

    public static final int motorId = 52;

  }

  public static final class OuttakeConstants {

    public static final int outtakeMotorId = 51;
    public static final int dislodgerMotorId = 52;

    // public static final double turnEncoderPositionFactor = 1;
    // public static final double turnEncoderVelocityFactor = 1;
    public static final int kIntakeSensorPort = 0;

    public static final double outtakeVoltage = 4;
    public static final double intakeVoltage = -2;
    public static final double dislodgerVoltage = -1;

  }

  public static final class LEDConstants {
    public static final int CANdleID = 40;
    public static final int ledCount = 200;
  }

  public static final class PassthroughConstants {

    public static double passthroughMotorVoltage = 6;
  }

  public static final class WristConstants{

      public static int wristMotorID = 53;

      public static final DCMotor wristGearbox = DCMotor.getNeoVortex(1);
      public static final double wristMotorReduction = 7.3; //  This number is arbitrary as freak


      public static double kDt = 0.02;

      public static double zeroOffset = 1.0 - ((5.05128918995 - (Math.PI / 2.0)) / (2.0 * Math.PI));

      public static double freeHangAngle = Math.PI / 2.0;

      public static double passthroughPositionRad = freeHangAngle;
      public static double L1PositionRad = Math.PI * 2.0 / 3.0;
      public static double L2L3PositionRad = Math.PI;
      public static double L4PositionRad = Math.PI * 2.8 / 2.0;

      public static double ksFeedforward = 0.15;
      public static double kvFeedforward = 0.6;

      public static double kpWrist = 3.0;
      public static double kiWrist = 0;
      public static double kdWrist = 0;

      public static double ksFeedforwardSim = 0;
      public static double kvFeedforwardSim = 0;

      public static double kpWristSim = 1;
      public static double kiWristSim = 0;
      public static double kdWristSim = 0;

      public static double maxVelocity = Math.PI * 3.5;
      public static double maxAcceleration = Math.PI * 2.25;


      // public static final double softUpperLimit = 4.07;
      // public static final double softLowerLimit = 0.19;
      public static final double softUpperLimitRotations = 4.51; // ~ 2pi * 0.7
      public static final double softLowerLimitRotations = Math.PI / 4.0;

  }

}
