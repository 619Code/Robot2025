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
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double kDt = 0.02;

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
    public static double maxSpeedMetersPerSec = 5.450;//5.450; //4.0; // was default
     public static final double angularVelocityMultiplier = 0.5;
    public static final double trackWidth = Units.inchesToMeters(22.8);
    public static final double wheelBase = Units.inchesToMeters(25.6);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    // public static final double possibleBumperWidth = 0.57912 + Units.inchesToMeters(3); // 0.6553199999999999
    // public static final double possibleBumperLength = 0.65024 + Units.inchesToMeters(3); //  0.72644


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

    public static final int frontLeftDriveCanId = 22;
    public static final int backLeftDriveCanId = 21;
    public static final int frontRightDriveCanId = 23;
    public static final int backRightDriveCanId = 20;

    public static final int frontLeftTurnCanId = 8;
    public static final int backLeftTurnCanId = 12;
    public static final int frontRightTurnCanId = 7;
    public static final int backRightTurnCanId = 2;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 40;
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveMotorReduction = 7.3; //(45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1); // DCMotor.getNeoVortex(1);

    //  Individual drive motor inversions
    public static final boolean frontLeftDriveMotorInverted = false;
    public static final boolean frontRightDriveMotorInverted = true;
    public static final boolean backLeftDriveMotorInverted = false;
    public static final boolean backRightDriveMotorInverted = true;

    //  Encoder data
    public static final int frontLeftDriveAbsoluteEncoderPort = 32;
    public static final double frontLeftDriveAbsoluteEncoderOffsetRots = -0.607910;
    public static final SensorDirectionValue frontLeftEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final int frontRightDriveAbsoluteEncoderPort = 33;
    public static final double frontRightDriveAbsoluteEncoderOffsetRots = -0.27050;
    public static final SensorDirectionValue frontRightEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final int backLeftDriveAbsoluteEncoderPort = 31;
    public static final double backLeftDriveAbsoluteEncoderOffsetRots = -0.02758;
    public static final SensorDirectionValue backLeftEncoderPositiveDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final int backRightDriveAbsoluteEncoderPort = 30;
    public static final double backRightDriveAbsoluteEncoderOffsetRots = 0.8620972027917303; //  Make sure the unit on this is correct

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

   public static final double robotMassKg = 56.699;
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

  } //  End drive


    //  NOTICE: None of these values have been tested. They are all arbitrary
  public static final class ElevatorConstants {

    public static final int leftMotorID = 54;
    public static final int rightMotorID = 55;


    public static final int maxVoltage = 10;

    public static final double wristMotorReduction = 1.0;


    public static final double maxVelocityMetersPerSec = 1.0; // Arbitrary
    public static final double maxAccelerationMetersPerSecSqrd = 1.5;











    //  Lerp thing

    public static final double minHeightMeters = Units.inchesToMeters(25);
    public static final double minHeightEncoderVal = 0;

    public static final double heightAtHigherPointMeters = Units.inchesToMeters(47.5);
    public static final double higherPointEncoderValue = 39.97153854370117;





    public static final double maxHeightMeters = Units.inchesToMeters(77);
    public static final double maxHeightEncoderVal = 68.75753021240234;















    public static final double encoderZeroOffsetRotations = 0;

    public static final double kpElevator = 9;
    public static final double kiElevator = 0.0;
    public static final double kdElevator = 0.0;

    public static final double ksFeedforward = 0.3;
    public static final double kvFeedforward = 4.4;

    public static final double feedforwardGravity = 0.40;

    public static final DCMotor elevatorGearbox = DCMotor.getNeoVortex(2);


    public enum ElevatorHeight{
      //  THESE ARE ALL STILL ARBITRARY
        HOME(minHeightMeters),
        FUNNEL(minHeightMeters + Units.inchesToMeters(5.0)), //  Used to be 8 inches

        L1(minHeightMeters + Units.inchesToMeters(2)),
        L2(minHeightMeters + Units.inchesToMeters(3)),
        L3(minHeightMeters + Units.inchesToMeters(3 + 16)),
        L4(minHeightMeters + Units.inchesToMeters(3 + 16 + 28 + 7));

        public final double heightMeters;
        ElevatorHeight(double _heightMeters){
            heightMeters = _heightMeters;
        }
    }
  } //  End elevator

  public static final class IntakeConstants {

    // NOTICE: None of these value have been tested as well.


    public static final class ExtensionMechanism{

      public static int extensionMotorId = 23;


      public static final double extendedPosition = 100.0;
      public static final double retractedPosition = 0.0;
      public static final double extensionTolerance = 5.0;
      public static final double half_stowPosition = 90.0;

      public static final double maxExtensionVoltage = 1.0;

      public static final double maxExtensionVelocity = 0;
      public static final double maxExtensionAcceleration = 0;

      public static final double extensionSoftLowerBound = 0;
      public static final double extensionSoftUpperBound = 200;

      public static final double kpIntakeExtension = 0.0;
      public static final double kiIntakeExtension = 0.0;
      public static final double kdIntakeExtension = 0.0;

      public static final double ksFeedforward = 0.0;
      public static final double kvFeedforward = 0.0;

      public static final double intakeMotorReduction = 7.3;  // ARBITRARY
      public static final DCMotor intakeGearbox = DCMotor.getNeoVortex(1);

    }

    public static final class Intake{

      public static int intakeMotorId = 3;

      public static final double intakingVoltage = 2.0;

    }
  } //  End intake

  public static final class ClimbConstants{
    // This values have not been tested.

    public static final int motorId = -1;  //  Don't know yet

    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;
    public static final double maxVoltage = 1.0;

    public static final double kpWrist = 0.0;
    public static final double kiWrist = 0.0;
    public static final double kdWrist = 0.0;

    public static final double ksFeedforward = 0.0;
    public static final double kvFeedforward = 0.0;


    public static  final double climbOutPosition = 90;
    public static final double climbInPosition = 0;

    public static final double climbSoftLowerBound = 0;
    public static final double climbSoftUpperBound = 200;
  } //  End climb

  public static final class ManipulatorConstants {

    public static final int outtakeMotorId = 51;
    public static final int dislodgerMotorId = 52;

    // public static final double turnEncoderPositionFactor = 1;
    // public static final double turnEncoderVelocityFactor = 1;
    public static final int kIntakeSensorPort = 9;

    public static final double outtakeVoltage = 4;
    public static final double intakeVoltage = -2;
    public static final double dislodgerVoltage = 3;


    public static final DCMotor outtakeMotorGearbox = DCMotor.getNeoVortex(1);
    public static final double outtakeMotorReduction = 1; //  This number is arbitrary as freak

    public static final DCMotor dislodgeMotorGearbox = DCMotor.getNeoVortex(1);
    public static final double dislodgeMotorReduction = 1; //  This number is arbitrary as freak

  } //  End manipulator

  public static final class LEDConstants {
    public static final int CANdleID = 40;
    public static final int ledCount = 200;
  } //  End LED

  public static final class PassthroughConstants {

    public static int leftMotorId = -1;
    public static int rightMotorId = -1;

    public static double passthroughMotorVoltage = 6;
  } //  End passthrough

  public static final class WristConstants{

      public static int wristMotorID = 53;

      public static int maxVoltage = 5;

      public static final DCMotor wristGearbox = DCMotor.getNeoVortex(1);
      public static final double wristMotorReduction = 7.3; //  This number is arbitrary as freak


      public static double zeroOffset = 1.0 - ((5.05128918995 - (Math.PI / 2.0)) / (2.0 * Math.PI));

      public static enum WristAngleRad {
        FREEHANG(Units.degreesToRadians(90)),
        FUNNEL_ANGLE(Units.degreesToRadians(66)),
        L1(Units.degreesToRadians(252)),  //  Needs to be changed
        L2L3(Units.degreesToRadians(252)),
        L4(Units.degreesToRadians(200));

        public final double positionRad;
        WristAngleRad(double _positionRad){
            positionRad = _positionRad;
        }
      }


      public static final class Control{
        public static double ksFeedforward = 0.15;
        public static double kvFeedforward = 0.6;

        public static double kpWrist = 3.0;
        public static double kiWrist = 0;
        public static double kdWrist = 0;
      }

      public static final class Constraints{
        public static final double softUpperLimitRadians = 4.51; // ~ 2pi * 0.7
        public static final double softLowerLimitRadians = Math.PI / 4.0;

        public static double maxVelocity = Math.PI * 3.5;
        public static double maxAcceleration = Math.PI * 2.25;
      }
  } //  End wrist

}
