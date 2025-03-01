package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.NTProfiledPIDF;

public class Intake extends SubsystemBase {

  private final SparkMax intakeMotor1;
  private final SparkMax intakeMotor2;
  private final SparkMax intakeExtensionMotor;

  private final NTProfiledPIDF extensionPID;
  private IntakeIO intakeIO;

  DoubleEntry kpextensionPIDEntry;
  DoubleEntry kiextensionPIDEntry;
  DoubleEntry kdextensionPIDEntry;
  DoubleEntry intakeExtensionTargetInDegreesEntry;
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

    // Motor Initialization and Configuration

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

    intakeMotor1.configure(config_1, null, PersistMode.kPersistParameters);
    intakeMotor2.configure(config_2, null, PersistMode.kPersistParameters);
    intakeExtensionMotor.configure(config_3, null, PersistMode.kPersistParameters);

    // PID Constraints and Initialization

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      Constants.IntakeConstants.maxVelocity,
      Constants.IntakeConstants.maxAcceleration);

    extensionPID = new NTProfiledPIDF("Intake",
    Constants.IntakeConstants.kp,
    Constants.IntakeConstants.ki,
    Constants.IntakeConstants.kd,
    Constants.IntakeConstants.ksFeedforward,
    Constants.IntakeConstants.kvFeedforward,
    constraints);

    // Initialization of NetworkTables

    intakeExtensionTargetInDegreesEntry = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionTargetInDegrees").getEntry(90);
    intakeExtensionMeasured = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionMeasured").getEntry(2);
    intakeExtensionVoltage = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionVoltage").getEntry(1);

    maxVelocityConstraint = NetworkTableInstance.getDefault().getDoubleTopic("maxVelocityConstraint").getEntry(20.0);
    maxAccConstraint = NetworkTableInstance.getDefault().getDoubleTopic("maxAccConstraint").getEntry(10.0);

    // Default Values

    intakeExtensionTargetInDegreesEntry.set(90);
    maxVelocityConstraint.set(20);
    maxAccConstraint.set(10);
  }

  @Override
  public void periodic() {

    // Get targetPosition and set PID goal

    double targetPosition = intakeExtensionTargetInDegreesEntry.get();
    extensionPID.setGoal(new State(targetPosition, 0));

    // IntakeIO current position

    IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    intakeIO.updateInputs(inputs);
    double currentPosition = inputs.intakePosition;

    // Voltage

    double voltage = extensionPID.calculate(currentPosition);
    voltage = Math.min(Math.max(voltage, -12.0), 12.0);

    intakeExtensionVoltage.set(voltage);
    intakeIO.ioPeriodic(voltage);
    intakeExtensionMeasured.set(inputs.intakePosition);
  }

  // Methods for specific positions

  public void goToPosition(double degrees){
    extensionPID.setGoal(new State(degrees, 0));
    intakeExtensionTargetInDegreesEntry.set(degrees);
  }

  public void goToExtendedPosition() {
    goToPosition(100);
  }

  public void goToRetractedPosition() {
    goToPosition(0);
  }

  // Check if Intake reached targetPosition

  public boolean hasReachedGoal(){
    return false; //extensionPID.hasReachedGoal();
  }
}
