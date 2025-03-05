package frc.robot.subsystems.WristStuff;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class WristIOSim implements WristIO {


    private final DCMotorSim wristMotor;


    public WristIOSim(){

        wristMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Constants.WristConstants.wristGearbox,
                0.025,
                Constants.WristConstants.wristMotorReduction),
            Constants.WristConstants.wristGearbox);

    }

    @Override
    public void setVoltage(double voltage){
        wristMotor.setInputVoltage(voltage);
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristPosition = wristMotor.getAngularPositionRad();

        wristMotor.update(0.02);
    }
}
