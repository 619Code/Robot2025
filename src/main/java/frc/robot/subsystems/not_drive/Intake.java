package frc.robot.subsystems.not_drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

  private final ProfiledPIDController extensionPID;
  private boolean seeking = false;

  private IntakeIO intakeIO;

  DoubleEntry kpextensionPIDEntry;
  DoubleEntry kiextensionPIDEntry;
  DoubleEntry kdextensionPIDEntry;
  DoubleEntry intakeExtensionTargetInDegreesEntry;

  //DoublePublisher intakeExtensionTargetInDegrees;
  DoublePublisher intakeExtensionMeasured;
  DoublePublisher intakeExtensionVoltage;

  DoubleEntry maxVelocityContraint;
  DoubleEntry maxAccConstraint;


  public Intake(int intakeExtensionMotorID) {
    if(Robot.isReal()){
      intakeIO = new intakeIOReal(intakeExtensionMotorID);
    }
    else{
      intakeIO = new intakeIOSim();
    }

    //extensionPID = new PIDController(0.0, 0.0, 0.0);
    //extensionPID = new ProfiledPIDController(kpextensionPIDEntry.get(), kiextensionPIDEntry.get(), kdextensionPIDEntry.get(), new TrapezoidProfile.Constraints(2,1));

    kpextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKp").getEntry(0.0);
    kiextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKi").getEntry(0.0);
    kdextensionPIDEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("IntakeKd").getEntry(0.0);

    intakeExtensionTargetInDegreesEntry = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionTargetInDegrees").getEntry(0);
    maxVelocityContraint = NetworkTableInstance.getDefault().getDoubleTopic("maxVelocityContraint").getEntry(2);
    maxAccConstraint = NetworkTableInstance.getDefault().getDoubleTopic("maxAccContraint").getEntry(1);

    intakeExtensionMeasured = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionMeasured").publish();
    intakeExtensionVoltage = NetworkTableInstance.getDefault().getDoubleTopic("intakeExtensionVoltage").publish();
    kpextensionPIDEntry.set(0);
    kiextensionPIDEntry.set(0);
    kdextensionPIDEntry.set(0);
    intakeExtensionTargetInDegreesEntry.set(0);
    maxVelocityContraint.set(2);
    maxAccConstraint.set(1);

    extensionPID = new ProfiledPIDController(kpextensionPIDEntry.get(), kiextensionPIDEntry.get(), kdextensionPIDEntry.get(), new TrapezoidProfile.Constraints(2,1));

  }

  @Override
  public void periodic() {

    extensionPID.setP(kpextensionPIDEntry.get());
    extensionPID.setI(kiextensionPIDEntry.get());
    extensionPID.setD(kdextensionPIDEntry.get());
    extensionPID.setConstraints(new TrapezoidProfile.Constraints(maxVelocityContraint.get(), maxAccConstraint.get()));

    // if (!seeking) return;

    // double voltage = extensionPID.calculate(intakeIO.getPosition());
    // voltage = Math.min(Math.max(voltage, -12.0), 12.0);

    // if (shouldStopSeeking()) {
    //   voltage = 0.0;
    // }
    double voltage = extensionPID.calculate(intakeIO.getPosition(), intakeExtensionTargetInDegreesEntry.get());

    intakeIO.setVoltage(voltage);
    intakeIO.update();

    intakeExtensionMeasured.set(intakeIO.getPosition());
    intakeExtensionVoltage.set(voltage);
  }

  // private boolean shouldStopSeeking() {
  //   return Math.abs(extensionPID.getSetpoint() - intakeIO.getPosition())
  //       <= Constants.IntakeConstants.extensionTolerance;
  // }

  public void goToPosition(double degrees)
  {
    intakeExtensionTargetInDegreesEntry.set(degrees);
  }

  // private void stopSeeking() {
  //   seeking = false;
  //   intakeIO.stopMotor();
  // }

  // private void startSeeking() {
  //   seeking = true;
  // }

  public void goToExtendedPosition() {
    //extensionPID.setSetpoint(Constants.IntakeConstants.extendedPosition);
    goToPosition(100);
    //startSeeking();
  }

  public void goToRetractedPosition() {
    goToPosition(0);
    //extensionPID.setSetpoint(Constants.IntakeConstants.retractedPosition);
    //startSeeking();
  }

}
