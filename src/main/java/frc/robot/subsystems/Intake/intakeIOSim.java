package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class intakeIOSim implements IntakeIO{
    private final SingleJointedArmSim armSim;
    //private double targetPosition;
    //private final double tolerance = 2.0;

    // Simulated Motor Initialization

    intakeIOSim(){
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

        // Arm State

        armSim.setState(Rotation2d.fromDegrees(90).getRadians(), 0);
        //targetPosition = 90;
    }

    // Update Intake Positon

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs){
        inputs.intakePosition = Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
    }

    // Set voltage and update time

    @Override
    public void ioPeriodic(double voltage){
        armSim.setInputVoltage(voltage);
        armSim.update(0.02);
    }
}
