package frc.robot.subsystems.Manipulator;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.IProfiledReset;

public class Manipulator extends SubsystemBase implements IProfiledReset {

  private final ManipulatorIO manipulatorIO;

  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();



  private final TrapezoidProfile dislodgerVoltageTrapezoidProfile;

  private State currentDislodgerSetpoint;
  private State currentDislodgerGoal;

  public Manipulator(){
    if(Robot.isReal()){
      manipulatorIO = new ManipulatorIOReal(
        Constants.ManipulatorConstants.outtakeMotorId,
        Constants.ManipulatorConstants.dislodgerMotorId
      );
    }
    else{
      manipulatorIO = new ManipulatorIOSim();
    }

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(15, 30);
    dislodgerVoltageTrapezoidProfile = new TrapezoidProfile(constraints);

    currentDislodgerSetpoint = new State(0, 0);
    currentDislodgerGoal = new State(0, 0);
  }


  @Override
  public void periodic() {
      if(Constants.currentMode == Mode.REPLAY){
          Logger.processInputs("RealOutputs/Manipulator", inputs);
          manipulatorIO.updateInputs(inputs);
      }else{
          manipulatorIO.updateInputs(inputs);
          Logger.processInputs("RealOutputs/Manipulator", inputs);
      }



      currentDislodgerSetpoint = dislodgerVoltageTrapezoidProfile.calculate(Constants.kDt, currentDislodgerSetpoint, currentDislodgerGoal);

      double voltage = currentDislodgerSetpoint.position;

      manipulatorIO.setDislodgerVoltage(voltage);

  }

  public boolean isDetectingCoral(){
    return inputs.hasCoral;
  }


  //  Outtake
  public void runOuttakeOut(){
    manipulatorIO.runOuttakeVoltage(Constants.ManipulatorConstants.outtakeVoltage);
  }
  public void runOuttakeIn(){
    manipulatorIO.runOuttakeVoltage(Constants.ManipulatorConstants.intakeVoltage);
  }
  public void runOuttakeInReverse(){
    manipulatorIO.runOuttakeVoltage(-Constants.ManipulatorConstants.intakeVoltage);
  }
  public void stopOuttake(){
    manipulatorIO.stopOuttake();
  }

  //  Dislodger
  public void startDislodger(boolean invert){
    currentDislodgerGoal = new State(
      Constants.ManipulatorConstants.dislodgerVoltage * (invert ? -1 : 1),
       5
    );
  }
  public void stopDislodger(){
    currentDislodgerGoal = new State(0, 0);
  }


  @Override
    public void ProfileReset() {
      // This one doesn't need to be reset. The only profiling is on the dislodger
    }
}
