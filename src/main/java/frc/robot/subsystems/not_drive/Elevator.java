package frc.robot.subsystems.not_drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  //  Current plan is to use the relative encoders

  private final SparkMax elevatorMotorL;
  private final SparkMax elevatorMotorR;

  private final RelativeEncoder lEncoder;

  private final PIDController elevatorPID;

  private boolean seeking = false;

  DoubleEntry kpelevatorPIDEntry;
  DoubleEntry kielevatorPIDEntry;
  DoubleEntry kdelevatorPIDEntry;

  public Elevator(int elevatorMotorID_L, int elevatorMotorID_R) {

    elevatorMotorL = new SparkMax(elevatorMotorID_L, MotorType.kBrushless);
    elevatorMotorR = new SparkMax(elevatorMotorID_R, MotorType.kBrushless);

    lEncoder = elevatorMotorL.getEncoder();

    SparkMaxConfig config_L = new SparkMaxConfig();
    config_L.idleMode(IdleMode.kBrake);

    SoftLimitConfig limitConfig = new SoftLimitConfig();
    limitConfig.forwardSoftLimit(Constants.ElevatorConstants.elevatorSoftUpperBound);
    limitConfig.reverseSoftLimit(Constants.ElevatorConstants.elevatorSoftLowerBound);

    config_L.softLimit.apply(limitConfig);

    SparkMaxConfig config_R = new SparkMaxConfig();
    config_R.idleMode(IdleMode.kBrake);
    config_R.follow(elevatorMotorL, true);

    elevatorMotorL.configure(config_L, null, null);
    elevatorMotorR.configure(config_R, null, null);

    //   For tuning

    kpelevatorPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("Elevatorkp").getEntry(0.0);
    kielevatorPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("Elevatorki").getEntry(0.0);
    kdelevatorPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("Elevatorkd").getEntry(0.0);

    kpelevatorPIDEntry.set(0.0);
    kielevatorPIDEntry.set(0.0);
    kdelevatorPIDEntry.set(0.0);

    elevatorPID = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {

    elevatorPID.setPID(
        kpelevatorPIDEntry.get(), kielevatorPIDEntry.get(), kdelevatorPIDEntry.get());

    if (!seeking) return;

    if (shouldStopSeeking()) {
      stopSeeking();
      return;
    }

    double voltage = elevatorPID.calculate(lEncoder.getPosition());
    voltage = Math.min(Math.max(voltage, -12.0), 12.0);  //  Clamp between -12 and 12

    elevatorMotorL.setVoltage(voltage);
  }

  private boolean shouldStopSeeking() {

    return Math.abs(elevatorPID.getSetpoint() - lEncoder.getPosition())
        <= Constants.ElevatorConstants.destinationTolerance;
  }

  private void stopSeeking() {

    seeking = false;
    elevatorMotorL.stopMotor();
  }

  private void startSeeking() {

    seeking = true;
  }

  public void goToHandoffHeight() {

    elevatorPID.setSetpoint(Constants.ElevatorConstants.handoffHeight);

    startSeeking();
  }

  public void goToL2Height() {
    elevatorPID.setSetpoint(Constants.ElevatorConstants.l2Height);
    startSeeking();
  }

  public void goToL3Height() {
    elevatorPID.setSetpoint(Constants.ElevatorConstants.l3Height);
    startSeeking();
  }

  public void goToL4Height() {
    elevatorPID.setSetpoint(Constants.ElevatorConstants.l4Height);
    startSeeking();
  }
}
