package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class intakeIOSim implements IntakeIO{
    private final SingleJointedArmSim armSim;
    private double position;

    @Override
    public double getPosition() {
        return Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
    }

    @Override
    public void stopMotor() {
        armSim.setInputVoltage(0);
    }

    @Override
    public void setVoltage(double voltage){
        armSim.setInputVoltage(voltage);
    }

    @Override
    public void update(){
        armSim.update(0.02);

        position = Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
    }

    intakeIOSim(){
        DCMotor motor = DCMotor.getNEO(1);
        armSim = new SingleJointedArmSim(motor,
        105,
        SingleJointedArmSim.estimateMOI(0.5, 6.0),
        0.5,
        Rotation2d.fromDegrees(0).getRadians(),
        Rotation2d.fromDegrees(90).getRadians(),
        true,
        Rotation2d.fromDegrees(90).getRadians()
        );
        armSim.setState(Rotation2d.fromDegrees(90).getRadians(), 0);
    }
}
