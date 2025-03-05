package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO{


    private final SingleJointedArmSim extensionMotor;

    private final DCMotorSim intakeMotor;


    public IntakeIOSim(){
        DCMotor motor = DCMotor.getNEO(1);
        extensionMotor = new SingleJointedArmSim(
            motor,
            105,
            SingleJointedArmSim.estimateMOI(0.5, 6.0),
            0.5,
            Rotation2d.fromDegrees(-20).getRadians(),
            Rotation2d.fromDegrees(110).getRadians(),
            true,
            Rotation2d.fromDegrees(90).getRadians()
        );
        extensionMotor.setState(Rotation2d.fromDegrees(90).getRadians(), 0);


        intakeMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Constants.IntakeConstants.ExtensionMechanism.intakeGearbox,
                0.025,
                Constants.IntakeConstants.ExtensionMechanism.intakeMotorReduction),
                Constants.IntakeConstants.ExtensionMechanism.intakeGearbox
            );
    }

    @Override
    public void stopExtensionMotor() {
        extensionMotor.setInputVoltage(0);
    }

    @Override
    public void setExtensionMotorVoltage(double voltage){
        extensionMotor.setInputVoltage(voltage);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs){
        inputs.intakeExtensionPosition = Rotation2d.fromRadians(extensionMotor.getAngleRads()).getDegrees();
        inputs.intakeMotorSpeedRadSec = intakeMotor.getAngularVelocityRadPerSec();
        

        extensionMotor.update(0.02);
        intakeMotor.update(0.02);
    }

    @Override
    public void setIntakeMotorVoltage(double voltage) {
        intakeMotor.setInputVoltage(voltage);
    }
}
