package frc.robot.subsystems.Intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.NTProfiledPIDF;

public class Intake extends SubsystemBase {

  private final NTProfiledPIDF extensionPID;
  private IntakeIO intakeIO;

  DoubleEntry intakeExtensionTargetInDegreesEntry;
  DoublePublisher intakeExtensionMeasured;
  DoublePublisher intakeExtensionVoltage;

  public Intake() {
    if(Robot.isReal()){
      intakeIO = new IntakeIOReal(Constants.IntakeConstants.intakeMotorId, Constants.IntakeConstants.extensionMotorId);
    }
    else{
      intakeIO = new IntakeIOSim();
    }

    intakeExtensionTargetInDegreesEntry = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionTargetInDegrees").getEntry(0);
    intakeExtensionMeasured = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionMeasured").getEntry(2);
    intakeExtensionVoltage = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionVoltage").getEntry(1);

    intakeExtensionMeasured = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionMeasured").publish();
    intakeExtensionVoltage = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionVoltage").publish();

    extensionPID = new NTProfiledPIDF(
      "Intake",
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      new Constraints(-1, -1)
    );

    intakeExtensionTargetInDegreesEntry.set(0);
  }

  @Override
  public void periodic() {

    double targetPosition = intakeExtensionTargetInDegreesEntry.get();
    extensionPID.setGoal(new State(targetPosition, 0));

    double voltage = extensionPID.calculate(intakeIO.getPosition());

    voltage = Math.min(Math.max(voltage, -12.0), 12.0);

    intakeIO.setExtensionMotorVoltage(voltage);

    // Log some stuff
    intakeExtensionMeasured.set(intakeIO.getPosition());
    intakeExtensionVoltage.set(voltage);
  }

  public void goToPosition(double degrees){
    intakeExtensionTargetInDegreesEntry.set(degrees);
  }

  public void goToExtendedPosition() {
    goToPosition(Constants.IntakeConstants.extendedPosition);
  }

  public void goToRetractedPosition() {
    goToPosition(Constants.IntakeConstants.retractedPosition);
  }

}
