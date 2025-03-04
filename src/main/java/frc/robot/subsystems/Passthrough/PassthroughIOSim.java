package frc.robot.subsystems.Passthrough;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class PassthroughIOSim implements PassthroughIO {

    DCMotorSim passthroughMotorL;
    DCMotorSim passthroughMotorR;

    public PassthroughIOSim(){

        passthroughMotorL = new DCMotorSim(null, null, null);
        passthroughMotorR = new DCMotorSim(null, null, null);

    }

    @Override
    public void setVoltage(double voltage){
        passthroughMotorL.setInputVoltage(voltage);
        passthroughMotorR.setInputVoltage(voltage);
    }

    @Override
    public void stopMotors(){
        passthroughMotorL.setInputVoltage(0);
        passthroughMotorR.setInputVoltage(0);
    }

    @Override
    public void updateInputs(PassthroughIOInputsAutoLogged inputs) {
        passthroughMotorL.getInputVoltage();
    }

    @Override
    public void periodic(){
        passthroughMotorL.update(Constants.PassthroughConstants.kDt);
    }
}
