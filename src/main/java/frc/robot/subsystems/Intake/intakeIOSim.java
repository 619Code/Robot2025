package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class intakeIOSim implements IntakeIO{
    private final SingleJointedArmSim armSim;
    private double targetPosition;
    private final double tolerance = 2.0;

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
        armSim.setState(Rotation2d.fromDegrees(90).getRadians(), 0);
        targetPosition = 90;
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs){
        inputs.intakePosition = Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
        inputs.intakeSetpointPosition = targetPosition;
    }

    @Override
    public void ioPeriodic(){
        double currentPos = Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
        double error = targetPosition - currentPos;

        double voltage = 0.1 * error;
        voltage = Math.max(Math.min(voltage, Constants.IntakeConstants.maxVoltage), -Constants.IntakeConstants.maxVoltage);
        armSim.setInputVoltage(voltage);
        armSim.update(0.02);
    }

    @Override
    public void setTargetPosition(double _position){
        targetPosition = _position;
    }

    @Override
    public boolean hasReachedGoal(){
        double currentPos = Rotation2d.fromRadians(armSim.getAngleRads()).getDegrees();
        return Math.abs(currentPos - targetPosition) < tolerance;
    }
}
