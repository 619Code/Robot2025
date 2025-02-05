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

package frc.robot.subsystems.drive;

// import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  //   private final SparkBase driveSpark;
  //   private final SparkBase turnSpark;
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final CANcoder absoluteTurnEncoder;
  private final PIDController turnController;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private final StatusSignal<Angle> absoluteTurnEncoderPositionSignal;
  private final StatusSignal<AngularVelocity> absoluteTurnEncoderVelocitySignal;

  public ModuleIOSpark(
      int module,
      boolean driveMotorInverted,
      boolean turnMotorInverted,
      boolean turnEncoderInverted,
      int absoluteEncoderCANId,
      double absoluteEncoderOffset,
      SensorDirectionValue positiveDirection) {

    zeroRotation =
        switch (module) {
          case 0 -> Constants.DriveConstants.frontLeftZeroRotation;
          case 1 -> Constants.DriveConstants.frontRightZeroRotation;
          case 2 -> Constants.DriveConstants.backLeftZeroRotation;
          case 3 -> Constants.DriveConstants.backRightZeroRotation;
          default -> new Rotation2d();
        };

    // driveSpark =
    //     new SparkFlex(
    //         switch (module) {
    //           case 0 -> frontLeftDriveCanId;
    //           case 1 -> frontRightDriveCanId;
    //           case 2 -> backLeftDriveCanId;
    //           case 3 -> backRightDriveCanId;
    //           default -> 0;
    //         },
    //         MotorType.kBrushless);

    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.DriveConstants.frontLeftDriveCanId;
              case 1 -> Constants.DriveConstants.frontRightDriveCanId;
              case 2 -> Constants.DriveConstants.backLeftDriveCanId;
              case 3 -> Constants.DriveConstants.backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);

    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.DriveConstants.frontLeftTurnCanId;
              case 1 -> Constants.DriveConstants.frontRightTurnCanId;
              case 2 -> Constants.DriveConstants.backLeftTurnCanId;
              case 3 -> Constants.DriveConstants.backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController =
        new PIDController(Constants.DriveConstants.turnKp, 0.0, Constants.DriveConstants.turnKd);
    turnController.enableContinuousInput(0, 2.0 * Math.PI);

    //  Initialize absolute turn encoder
    absoluteTurnEncoder = new CANcoder(absoluteEncoderCANId);
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
    canCoderConfiguration.MagnetSensor.SensorDirection = positiveDirection;
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    // canCoderConfiguration.primaryEncoderPositionPeriodMs
    // absoluteEncoder.configAllSettings(canCoderConfiguration);
    absoluteTurnEncoder.getConfigurator().apply(canCoderConfiguration);

    absoluteTurnEncoderPositionSignal = absoluteTurnEncoder.getAbsolutePosition();
    absoluteTurnEncoderVelocitySignal = absoluteTurnEncoder.getVelocity();

    absoluteTurnEncoderPositionSignal.setUpdateFrequency(500);
    absoluteTurnEncoderVelocitySignal.setUpdateFrequency(500);

    // Configure drive motor
    // SparkFlexConfig driveConfig = new SparkFlexConfig();
    // driveConfig
    //     //     .inverted(driveMotorInverted)
    //     .idleMode(IdleMode.kBrake)
    //     .smartCurrentLimit(driveMotorCurrentLimit)
    //     .voltageCompensation(12.0);
    // driveConfig
    //     .encoder
    //     //    .inverted(driveEncoderInverted)
    //     .positionConversionFactor(driveEncoderPositionFactor)
    //     .velocityConversionFactor(driveEncoderVelocityFactor)
    //     .uvwMeasurementPeriod(10)
    //     .uvwAverageDepth(2);
    // driveConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pidf(
    //         driveKp, 0.0,
    //         driveKd, 0.0);
    // driveConfig
    //     .signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);
    // tryUntilOk(
    //     driveSpark,
    //     5,
    //     () ->
    //         driveSpark.configure(
    //             driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    // tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(driveMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0)
        .closedLoopRampRate(0.1);
    driveConfig
        .encoder
        .positionConversionFactor(Constants.DriveConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(Constants.DriveConstants.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Constants.DriveConstants.driveKp, 0.0, Constants.DriveConstants.driveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.turnMotorCurrentLimit)
        .voltageCompensation(12.0)
        .closedLoopRampRate(0.1);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(Constants.DriveConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(Constants.DriveConstants.turnEncoderVelocityFactor)
        .averageDepth(2);
    // turnConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //     .positionWrappingEnabled(true)
    //     .positionWrappingInputRange(
    //         Constants.DriveConstants.turnPIDMinInput, Constants.DriveConstants.turnPIDMaxInput)
    //     .pidf(Constants.DriveConstants.turnKp, 0.0, Constants.DriveConstants.turnKd, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(
            (int) (1000.0 / Constants.DriveConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(turnSpark, this::GetAbsoluteTurnEncoderPositionRad);
  }

  double GetAbsoluteTurnEncoderPositionRad() {
    return absoluteTurnEncoderPositionSignal.refresh().getValueAsDouble() * 2 * Math.PI;
  }

  double GetAbsoluteTurnEncoderVelRotationsPerMinute() {
    return absoluteTurnEncoderVelocitySignal.refresh().getValueAsDouble() * 60.0;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        this::GetAbsoluteTurnEncoderPositionRad,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(
        turnSpark,
        this::GetAbsoluteTurnEncoderVelRotationsPerMinute,
        (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // TODO: This breaks convention
    // We are updating the feedback and setting the motor output.
    // This is because this is the only function we can rely on being called periodically.
    turnSpark.set(turnController.calculate(GetAbsoluteTurnEncoderPositionRad()));

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        Constants.DriveConstants.driveKs * Math.signum(velocityRadPerSec)
            + Constants.DriveConstants.driveKv * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(),
            Constants.DriveConstants.turnPIDMinInput,
            Constants.DriveConstants.turnPIDMaxInput);
    // turnController.setReference(setpoint, ControlType.kPosition);

    turnController.setSetpoint(setpoint);
  }

  //   @Override
  //   public void setTurnMotorPID(double kp, double kd) {
  //     turnController.setPID(kp, 0.0, kd);
  //   }
}
