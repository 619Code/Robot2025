package frc.robot.subsystems.Climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO{
    private final SingleJointedArmSim armSim;
    //private double targetPosition;

    // Simulated Motor Initialization

    public ClimbIOSim() {
        DCMotor motor = DCMotor.getNEO(1);
        armSim = new SingleJointedArmSim(
            motor,
            105,
            SingleJointedArmSim.estimateMOI(0.5, 6.0),
            0.5,
            Rotation2d.fromDegrees(-20).getRadians(),
            Rotation2d.fromDegrees(200).getRadians(),
            true,
            Rotation2d.fromDegrees(180).getRadians()
        );

        // Arm State

        armSim.setState(Rotation2d.fromDegrees(180).getRadians(), 0);
    }

    // Update Climb Position

    @Override
    public void updateInputs(ClimbIO.ClimbIOInputs inputs){
        inputs.ClimbPosition = Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
    }

    // Set voltage and update time

    @Override
    public void ioPeriodic(double voltage){
        armSim.setInputVoltage(voltage);
        armSim.update(0.02);
    }
}
