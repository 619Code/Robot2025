package frc.robot.subsystems.Manipulator;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Outtake.OuttakeIOInputsAutoLogged;
import frc.robot.Robot;

public class Manipulator extends SubsystemBase {

  private final ManipulatorIO manipulatorIO;

  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();



  private final TrapezoidProfile dislodgerVoltageTrapezoidProfile;

  private State currentDislodgerSetpoint;
  private State currentDislodgerGoal;

  public Manipulator(){
    if(Robot.isReal()){
      manipulatorIO = new ManipulatorIOReal(
        Constants.OuttakeConstants.outtakeMotorId, 
        Constants.OuttakeConstants.dislodgerMotorId
      );
    }
    else{
      for(int i = 0; i < 100; i++){
        System.out.println("NEED TO CREATE THE MANIPULATOR SIMULATION");
      }
      manipulatorIO = null;
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



      currentDislodgerSetpoint = dislodgerVoltageTrapezoidProfile.calculate(0.02, currentDislodgerSetpoint, currentDislodgerGoal);

      double voltage = currentDislodgerSetpoint.position;

      manipulatorIO.setDislodgerVoltage(voltage);

  }

  public boolean isDetectingCoral(){
    return inputs.hasCoral;
  }


  //  Outtake
  public void runOuttakeOut(){
    manipulatorIO.runOuttakeVoltage(Constants.OuttakeConstants.outtakeVoltage);
  }
  public void runOuttakeIn(){
    manipulatorIO.runOuttakeVoltage(Constants.OuttakeConstants.intakeVoltage);
  }
  public void stopOuttake(){
    manipulatorIO.stopOuttake();
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
