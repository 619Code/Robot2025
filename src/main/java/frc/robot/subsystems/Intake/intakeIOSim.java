package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO{


    private final SingleJointedArmSim armSim;


    public IntakeIOSim(){
        DCMotor motor = DCMotor.getNEO(1);
        armSim = new SingleJointedArmSim(motor,
        105,
        SingleJointedArmSim.estimateMOI(0.5, 6.0),
        0.5,
        Rotation2d.fromDegrees(-20).getRadians(),
        Rotation2d.fromDegrees(110).getRadians(),
        true,
        Rotation2d.fromDegrees(90).getRadians()
        );
        armSim.setState(Rotation2d.fromDegrees(90).getRadians(), 0);
    }

    @Override
    public double getPosition() {
        return Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
    }

    @Override
    public void stopMotor() {
        armSim.setInputVoltage(0);
    }

    @Override
    public void setExtensionMotorVoltage(double voltage){
        armSim.setInputVoltage(voltage);
    }

    // @Override
    // public void updateInputs(IntakeIOInputsAutoLogged inputs){
    //     inputs.position = Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();

    //     armSim.update(0.02);
    // }

    @Override
    public void setIntakeMotorVoltage(double voltage) {
        throw new UnsupportedOperationException();
    }
}
