package frc.robot.subsystems.not_drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

  private final SparkMax intakeMotor1;
  private final SparkMax intakeMotor2;

  private final PIDController extensionPID;

  private boolean seeking = false;

  private IntakeIO intakeIO;

  DoubleEntry kpextensionPIDEntry;
  DoubleEntry kiextensionPIDEntry;
  DoubleEntry kdextensionPIDEntry;
  DoublePublisher intakeExtensionSetpoint;
  DoublePublisher intakeExtensionMeasured;
  DoublePublisher intakeExtensionVoltage;

  new TrapezoidProfile.State(5, 0);
  new TrapezoidProfile.Constraints(10, 20);

  public Intake(int intakeMotorID_1, int intakeMotorID_2, int intakeExtensionMotorID) {
    if(Robot.isReal()){
      intakeIO = new intakeIOReal(intakeExtensionMotorID);
    }
    else{
      intakeIO = new intakeIOSim();
    }

    intakeMotor1 = new SparkMax(intakeMotorID_1, MotorType.kBrushless);
    intakeMotor2 = new SparkMax(intakeMotorID_2, MotorType.kBrushless);

    SparkMaxConfig config_1 = new SparkMaxConfig();
    config_1.idleMode(IdleMode.kBrake);

    SparkMaxConfig config_2 = new SparkMaxConfig();
    config_2.idleMode(IdleMode.kBrake);
    config_2.follow(intakeMotor1, false);

    SparkMaxConfig config_3 = new SparkMaxConfig();
    config_3.idleMode(IdleMode.kBrake);

    SoftLimitConfig limitConfig = new SoftLimitConfig();
    limitConfig.forwardSoftLimit(Constants.IntakeConstants.intakeSoftUpperBound);
    limitConfig.reverseSoftLimit(Constants.IntakeConstants.intakeSoftLowerBound);

    config_3.softLimit.apply(limitConfig);

    intakeMotor1.configure(config_1, null, null);
    intakeMotor2.configure(config_2, null, null);

    extensionPID = new PIDController(0.0, 0.0, 0.0);

    kpextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKp").getEntry(0.0);
    kiextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKi").getEntry(0.0);
    kdextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKd").getEntry(0.0);
    intakeExtensionSetpoint = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionSetpoint").publish();
    intakeExtensionMeasured = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionMeasured").publish();
    intakeExtensionVoltage = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionVoltage").publish();
    kpextensionPIDEntry.set(0);
    kiextensionPIDEntry.set(0);
    kdextensionPIDEntry.set(0);
  }

  @Override
  public void periodic() {

    extensionPID.setP(kpextensionPIDEntry.get());
    extensionPID.setI(kiextensionPIDEntry.get());
    extensionPID.setD(kdextensionPIDEntry.get());

    if (!seeking) return;

    double voltage = extensionPID.calculate(intakeIO.getPosition());
    voltage = Math.min(Math.max(voltage, -12.0), 12.0);

    if (shouldStopSeeking()) {
      voltage = 0.0;
    }

    intakeIO.setVoltage(voltage);
    intakeIO.update();
    intakeExtensionSetpoint.set(extensionPID.getSetpoint());
    intakeExtensionMeasured.set(intakeIO.getPosition());
    intakeExtensionVoltage.set(voltage);
  }

  private boolean shouldStopSeeking() {
    return Math.abs(extensionPID.getSetpoint() - intakeIO.getPosition())
        <= Constants.IntakeConstants.extensionTolerance;
  }

  private void stopSeeking() {
    seeking = false;
    intakeIO.stopMotor();
  }

  private void startSeeking() {
    seeking = true;
  }

  public void goToExtendedPosition() {
    extensionPID.setSetpoint(Constants.IntakeConstants.extendedPosition);
    
    startSeeking();
  }

  public void goToRetractedPosition() {
    extensionPID.setSetpoint(Constants.IntakeConstants.retractedPosition);
    startSeeking();
  }

}
