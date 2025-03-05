package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.util.NTProfiledPIDF;

public class Intake extends SubsystemBase {

  private final NTProfiledPIDF extensionPID;
  private IntakeIO intakeIO;


  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake() {
    if(Robot.isReal()){
      intakeIO = new IntakeIOReal(Constants.IntakeConstants.intakeMotorId, Constants.IntakeConstants.extensionMotorId);
    }
    else{
      intakeIO = new IntakeIOSim();
    }

   
    extensionPID = new NTProfiledPIDF(
      "Intake",
      Constants.IntakeConstants.kpIntakeExtension,
      Constants.IntakeConstants.kiIntakeExtension,
      Constants.IntakeConstants.kdIntakeExtension,
      Constants.IntakeConstants.ksFeedforward,
      Constants.IntakeConstants.kvFeedforward,
      new Constraints(Constants.IntakeConstants.maxVelocity, Constants.IntakeConstants.maxAcceleration)
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

    double voltage = extensionPID.calculate(inputs.intakePosition);

    voltage = Math.min(Math.max(voltage, -Constants.IntakeConstants.maxVoltage), Constants.IntakeConstants.maxVoltage);

    intakeIO.setExtensionMotorVoltage(voltage);

  }

  public void goToPosition(double degrees){
    extensionPID.setGoal(new State(degrees, 0));
  }

  public void goToExtendedPosition() {
    goToPosition(Constants.IntakeConstants.extendedPosition);
  }

  public void goToRetractedPosition() {
    goToPosition(Constants.IntakeConstants.retractedPosition);
  }

}
