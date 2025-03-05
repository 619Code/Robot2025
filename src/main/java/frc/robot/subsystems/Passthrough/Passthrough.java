package frc.robot.subsystems.Passthrough;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;


public class Passthrough extends SubsystemBase {

  private PassthroughIO passthroughIO;


  private PassthroughIOInputsAutoLogged inputs = new PassthroughIOInputsAutoLogged();


  public Passthrough(int passthroughMotorID_L, int passthroughMotorID_R) {
    if(Robot.isReal()){
      passthroughIO = new PassthroughIOReal(passthroughMotorID_L, passthroughMotorID_R);
    }
    else{
      passthroughIO = new PassthroughIOSim();
    }
  }
  // Need to connect to Intake Start and Outake Sensor.

  @Override
  public void periodic(){
    if(Constants.currentMode == Mode.REPLAY){
          Logger.processInputs("RealOutputs/Passthrough", inputs);
          passthroughIO.updateInputs(inputs);
      }else{
          passthroughIO.updateInputs(inputs);
          Logger.processInputs("RealOutputs/Passthrough", inputs);
      }
  }

  //  The below two functions may not need to be public (can decide when we know the sensor
  // situation)
  public void RunPassthrough() {
    passthroughIO.setVoltage(Constants.PassthroughConstants.passthroughMotorVoltage);
  }

  public void HaltPassthrough() {
    passthroughIO.stopMotors();
  }
}
