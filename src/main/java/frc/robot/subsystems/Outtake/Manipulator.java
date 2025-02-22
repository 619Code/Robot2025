package frc.robot.subsystems.Outtake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class Manipulator extends SubsystemBase {

  private final ManipulatorIO io;

  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  public Manipulator(ManipulatorIO _io){
    io = _io;
  }


  @Override
  public void periodic() {

      if(Constants.currentMode == Mode.REPLAY){
          Logger.processInputs("Manipulator", inputs);
          io.updateInputs(inputs);
      }else{
          io.updateInputs(inputs);
          Logger.processInputs("Manipulator", inputs);
      }

      io.ioPeriodic();

  }

  public boolean isDetectingCoral(){
    return inputs.hasCoral;
  }


  //  Outtake
  public void runOuttakeOut(){
    io.runOuttakeOut();
  }
  public void runOuttakeIn(){
    io.runOuttakeIn();
  }
  public void stopOuttake(){
    io.stopOuttake();
  }

  //  Dislodger
  public void runDislodger(){
    io.runDislodger();
  }
  public void stopDislodger(){
    io.stopDislodger();
  }
}
