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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ManipulatorConstants {
    public static final int kIntakeLeaderPort = 60;  //20;  Test bed  value
    public static final boolean kInakeLeaderInverted = false;
    public static final int kShooterLeaderPort = 54; //10; TEst bed value
    public static final boolean kShooterLeaderInverted = true;

    public static final double intakeSpeed = 0.3;
    public static final double intakeSpeedWhenOuttaking = 1;//0.6;
    public static final double outtakeSpeedSpeaker = 1;
    public static final double outtakeSpeedAmp = 1;

    public static final double outtakeSpeedSpeakerVoltage = 12;
    public static final double outtakeSpeedAmpVoltage = 12;

    public static final int kIntakeSensorPort = 9;  //0f test bed value

    public static final int speakerShooterVelocityToReachBeforeFeedingNote = 3500;//3000;
    public static final int passerShooterVelocityToReachBeforeFeedingNote = 5000;//3000
    public static final int ampShooterVelocityToReachBeforeFeedingNote = 2000;

    //Shooter PID
    public static final double SHOOTER_KP = 0.0023237;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KS = 0.18955;
    public static final double SHOOTER_KV = 0.15861;//.13861
    public static final double SHOOTER_KA = 0.0093339;
    public static final double SHOOTER_MAX_RPM = 5600;
    public static final double SHOOTER_MAX_OUTPUT = 1;
    public static final double SHOOTER_MIN_OUTPUT = 0;

    public static final double shooterIdleRPM = 1500;
    
}

public static final class HingeConstants {
  public static final int kHingeLeaderPort = 52; // left motor
  public static final int kHingeFollowerPort = 61; // right motor
  public static final boolean kHingeLeaderInverted = false;
  public static final boolean kHingeFollowerInverted = true;

  public static final int kAbsoluteEncoderPort = 0;
  public static final int kAbsoluteEncoderOffset = 0;
  public static final double kMaxAngle = 128; //TEMP VALUE
  public static final double kMinAngle = 63; //TEMP VALUE

  public static final float topEncoderSoftLimit = 73;
  public static final float bottomEncoderSoftLimit = -0.01f;
  
  public static final double kHingeP = 0.27; //UNTUNED
  public static final double kHingeI = 0; //UNTUNED
  public static final double kHingeD = 0; //UNTUNED
  public static final double kHingeG = .14; 
  public static final double kHingeV = 5.35; 
  public static final double kHingeA = 0.0; 
  public static final double kHingeS = 0.0;

  public static final double kHingeMaxVelocityRadPerSecond = 0;
  public static final double KHingeMaxAccelerationRadPerSecond = 0;

  public static final double kIntakeAngle = 110.4;
  public static final double kShootingAngle = Constants.HingeConstants.kMinAngle;
  public static final double kAmpAngle = 127.5;

  public static final double rawEncoderLow = .88;
  public static final double rawEncoderHigh = .617;
  public static final double degreesLow = 60;
  public static final double degreesHigh = 128;
  public static final double kLongShotAngle = 70;

  public static final boolean kHingeEnabled = true;

  }

}
