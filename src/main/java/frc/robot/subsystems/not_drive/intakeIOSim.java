package frc.robot.subsystems.not_drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class intakeIOSim implements IntakeIO{
    private final DCMotorSim simMotor;

    @Override
    public double getPosition() {
        return simMotor.getAngularPositionRotations();
    }

    @Override
    public void stopMotor() {
        simMotor.setInputVoltage(0);
    }

    @Override
    public void setVoltage(double voltage){
        simMotor.setInputVoltage(voltage);
    }

    @Override
    public void update(){
        simMotor.update(0.02);
    }

    intakeIOSim(){
        DCMotor motor = DCMotor.getNEO(1);
        simMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor,
                0.000025,
                1.0/105
                ),
            motor
        );
    }
}
