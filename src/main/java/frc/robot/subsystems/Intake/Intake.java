package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
  private final SparkMax intakeExtensionMotor;

  private final ProfiledPIDController extensionPID;
  private IntakeIO intakeIO;

  DoubleEntry kpextensionPIDEntry;
  DoubleEntry kiextensionPIDEntry;
  DoubleEntry kdextensionPIDEntry;
  DoubleEntry intakeExtensionTargetInDegressEntry;
  DoublePublisher intakeExtensionMeasured;
  DoublePublisher intakeExtensionVoltage;

  DoubleEntry maxVelocityConstraint;
  DoubleEntry maxAccConstraint;

  public Intake(int intakeMotorID_1, int intakeMotorID_2, int intakeExtensionMotorID) {
    if(Robot.isReal()){
      intakeIO = new intakeIOReal(intakeExtensionMotorID);
    }
    else{
      intakeIO = new intakeIOSim();
    }

    intakeMotor1 = new SparkMax(intakeMotorID_1, MotorType.kBrushless);
    intakeMotor2 = new SparkMax(intakeMotorID_2, MotorType.kBrushless);
    intakeExtensionMotor = new SparkMax(intakeExtensionMotorID, MotorType.kBrushless);

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
    intakeExtensionMotor.configure(config_3, null, null);

    kpextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKp").getEntry(0.0);
    kiextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKi").getEntry(0.0);
    kdextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKd").getEntry(0.0);
    intakeExtensionTargetInDegressEntry = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionTargetInDegrees").getEntry(0);
    intakeExtensionMeasured = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionMeasured").getEntry(2);
    intakeExtensionVoltage = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionVoltage").getEntry(1);

    intakeExtensionMeasured = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionMeasured").publish();
    intakeExtensionVoltage = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionVoltage").publish();

    maxVelocityConstraint = NetworkTableInstance.getDefault().getDoubleTopic("maxVelocityConstraint").getEntry(2.0);
    maxAccConstraint = NetworkTableInstance.getDefault().getDoubleTopic("maxAccConstraint").getEntry(1.0);

    extensionPID = new ProfiledPIDController(kpextensionPIDEntry.get(), kiextensionPIDEntry.get(), kdextensionPIDEntry.get(),
    new TrapezoidProfile.Constraints(maxVelocityConstraint.get(), maxAccConstraint.get()));

    maxVelocityConstraint.set(2.0);
    maxAccConstraint.set(1.0);

    kpextensionPIDEntry.set(0);
    kiextensionPIDEntry.set(0);
    kdextensionPIDEntry.set(0);
    intakeExtensionTargetInDegressEntry.set(0);
    maxVelocityConstraint.set(2);
    maxAccConstraint.set(1);
  }

  @Override
  public void periodic() {

    extensionPID.setP(kpextensionPIDEntry.get());
    extensionPID.setI(kiextensionPIDEntry.get());
    extensionPID.setD(kdextensionPIDEntry.get());
    extensionPID.setConstraints(new TrapezoidProfile.Constraints(maxVelocityConstraint.get(), maxAccConstraint.get()));


    double voltage = extensionPID.calculate(intakeIO.getPosition(), intakeExtensionTargetInDegressEntry.get());
    voltage = Math.min(Math.max(voltage, -12.0), 12.0);

    intakeIO.setVoltage(voltage);
    intakeIO.update();

    intakeExtensionMeasured.set(intakeIO.getPosition());
    intakeExtensionVoltage.set(voltage);
  }

  public void goToPosition(double degrees){
    intakeExtensionTargetInDegressEntry.set(degrees);
  }

  public void goToExtendedPosition() {
    goToPosition(100);
  }

  public void goToRetractedPosition() {
    goToPosition(0);
  }

}
