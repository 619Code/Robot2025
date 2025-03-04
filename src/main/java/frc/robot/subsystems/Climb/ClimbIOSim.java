package frc.robot.subsystems.Climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO{
    private final SingleJointedArmSim armSim;

    @Override
    public double getPosition(){
        return Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
    }

    @Override
    public void stopMotor(){
        armSim.setInputVoltage(0);
    }

    @Override
    public void setVoltage(double voltage){
        armSim.setInputVoltage(voltage);
    }

    // @Override
    // public void updateInputs(){

    //     //  CONTINUE HERE
    //     armSim.update(0.02);
    // }

    public ClimbIOSim(int climbMotorID) {
        DCMotor motor = DCMotor.getNEO(1);
        armSim = new SingleJointedArmSim(
            motor,
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
}
