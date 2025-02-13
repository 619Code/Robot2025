package frc.robot.subsystems.not_drive.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class intakeArmIOSim implements IntakeArmIO{
    //private final DCMotorSim simMotor;
    private final SingleJointedArmSim armSim;

    @Override
    public double getPosition() {
        //return simMotor.getAngularPositionRotations() * gearRatio;
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
    }

    public intakeArmIOSim(){
        DCMotor motor = DCMotor.getNEO(1);
        armSim = new SingleJointedArmSim(
            motor,
            105,
            SingleJointedArmSim.estimateMOI(0.5,6),
            0.5,
            Rotation2d.fromDegrees(0).getRadians(),
            Rotation2d.fromDegrees(90).getRadians(),
            true,
            Rotation2d.fromDegrees(90).getRadians());

        armSim.setState(Rotation2d.fromDegrees(90).getRadians(), 0);
    }
}
