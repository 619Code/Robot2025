package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.IProfiledReset;
import frc.robot.util.Help;
import frc.robot.util.NTProfiledPIDF;

public class Intake extends SubsystemBase implements IProfiledReset {

  private final NTProfiledPIDF extensionPID;
  private IntakeIO intakeIO;


  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake() {
    if(Robot.isReal()){
      intakeIO = new IntakeIOReal(
        Constants.IntakeConstants.Intake.intakeMotorId,
        Constants.IntakeConstants.ExtensionMechanism.extensionMotorId
      );
    }
    else{
      intakeIO = new IntakeIOSim();
    }


    extensionPID = new NTProfiledPIDF(
      "Intake",
      Constants.IntakeConstants.ExtensionMechanism.kpIntakeExtension,
      Constants.IntakeConstants.ExtensionMechanism.kiIntakeExtension,
      Constants.IntakeConstants.ExtensionMechanism.kdIntakeExtension,
      Constants.IntakeConstants.ExtensionMechanism.ksFeedforward,
      Constants.IntakeConstants.ExtensionMechanism.kvFeedforward,
      new Constraints(
            Constants.IntakeConstants.ExtensionMechanism.maxExtensionVelocity,
            Constants.IntakeConstants.ExtensionMechanism.maxExtensionAcceleration
      )
    );

  }

  @Override
  public void periodic() {
    if(Constants.currentMode == Mode.REPLAY){
        Logger.processInputs("RealOutputs/Climb", inputs);
        intakeIO.updateInputs(inputs);
    }else{
      intakeIO.updateInputs(inputs);
        Logger.processInputs("RealOutputs/Climb", inputs);
    }

    double voltage = extensionPID.calculate(getPosition());

    voltage = Help.clamp(
      voltage,
      -Constants.IntakeConstants.ExtensionMechanism.maxExtensionVoltage,
      Constants.IntakeConstants.ExtensionMechanism.maxExtensionVoltage
    );

    intakeIO.setExtensionMotorVoltage(voltage);

  }

  public void goToPosition(double degrees){
    extensionPID.setGoal(new State(degrees, 0));
  }

  public void goToExtendedPosition() {
    goToPosition(Constants.IntakeConstants.ExtensionMechanism.extendedPosition);
  }

  public void goToRetractedPosition() {
    goToPosition(Constants.IntakeConstants.ExtensionMechanism.retractedPosition);
  }

  public void runIntake(){
    intakeIO.setIntakeMotorVoltage(Constants.IntakeConstants.Intake.intakingVoltage);
  }

  public void stopIntake(){
    intakeIO.setIntakeMotorVoltage(0);
  }


  private double getPosition(){
    return inputs.intakeExtensionPosition;
  }


  @Override
  public void ProfileReset() {
      extensionPID.setGoal(new State(getPosition(), 0));
  }
}
