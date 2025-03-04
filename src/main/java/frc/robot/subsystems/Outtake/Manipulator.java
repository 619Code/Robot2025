package frc.robot.subsystems.Outtake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class Manipulator extends SubsystemBase {

  private final ManipulatorIO io;

  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();



  private final TrapezoidProfile dislodgerTrapezoidProfile;

  private State currentDislodgerSetpoint;
  private State currentDislodgerGoal;

  public Manipulator(ManipulatorIO _io){
    io = _io;

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(15, 30);
    dislodgerTrapezoidProfile = new TrapezoidProfile(constraints);

    currentDislodgerSetpoint = new State(0, 0);
    currentDislodgerGoal = new State(0, 0);
  }


  @Override
  public void periodic() {
      if(Constants.currentMode == Mode.REPLAY){
          Logger.processInputs("RealOutputs/Manipulator", inputs);
          io.updateInputs(inputs);
      }else{
          io.updateInputs(inputs);
          Logger.processInputs("RealOutputs/Manipulator", inputs);
      }



      currentDislodgerSetpoint = dislodgerTrapezoidProfile.calculate(Constants.WristConstants.kDt, currentDislodgerSetpoint, currentDislodgerGoal);

      double voltage = currentDislodgerSetpoint.position;

      io.setDislodgerVoltage(voltage);

  }

  public boolean isDetectingCoral(){
    return inputs.hasCoral;
  }


  //  Outtake
  public void runOuttakeOut(){
    io.runOuttakeVoltage(Constants.OuttakeConstants.outtakeVoltage);
  }
  public void runOuttakeIn(){
    io.runOuttakeVoltage(Constants.OuttakeConstants.intakeVoltage);
  }
  public void stopOuttake(){
    io.stopOuttake();
  }

  //  Dislodger
  public void startDislodger(boolean invert){
    currentDislodgerGoal = new State(
      Constants.OuttakeConstants.dislodgerVoltage * (invert ? -1 : 1),
       0
    );
  }
  public void stopDislodger(){
    currentDislodgerGoal = new State(0, 0);
  }
}
