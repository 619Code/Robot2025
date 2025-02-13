package frc.robot.subsystems.not_drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.not_drive.motors.IntakeArmIO;
import frc.robot.subsystems.not_drive.motors.intakeArmIOReal;
import frc.robot.subsystems.not_drive.motors.intakeArmIOSim;

public class Intake extends SubsystemBase {

  private final ProfiledPIDController extensionPID;
  //private boolean seeking = false;

  private IntakeArmIO intakeArmIO;

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
      intakeArmIO = new intakeArmIOReal(intakeExtensionMotorID);
    }
    else{
      intakeArmIO = new intakeArmIOSim();
    }

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

    double voltage = extensionPID.calculate(intakeArmIO.getPosition(), intakeExtensionTargetInDegreesEntry.get());

    intakeArmIO.setVoltage(voltage);
    intakeArmIO.update();

    intakeExtensionMeasured.set(intakeArmIO.getPosition());
    intakeExtensionVoltage.set(voltage);
  }

  public void goToPosition(double degrees)
  {
    intakeExtensionTargetInDegreesEntry.set(degrees);
  }

  public void goToExtendedPosition() {
    goToPosition(100);
  }

  public void goToRetractedPosition() {
    goToPosition(0);
  }

}
