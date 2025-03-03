package frc.robot.subsystems.Passthrough;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class Passthrough extends SubsystemBase {

  private PassthroughIO io;


  public Passthrough(int passthroughMotorID_L, int passthroughMotorID_R) {
    if(Robot.isReal()){
      io = new PassthroughIOReal(passthroughMotorID_L, passthroughMotorID_R);
    }
    else{
      io = new PassthroughIOSim();
    }

  }
  // Need to connect to Intake Start and Outake Sensor.

  @Override
  public void periodic(){
    io.ioPeriodic();
  }

  //  The below two functions may not need to be public (can decide when we know the sensor
  // situation)
  public void RunPassthrough() {
    io.setVoltage(Constants.PassthroughConstants.passthroughMotorVoltage);
  }

  public void HaltPassthrough() {
    io.stopMotors();
  }
}
