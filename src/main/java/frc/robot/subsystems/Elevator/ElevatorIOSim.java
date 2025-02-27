package frc.robot.subsystems.Elevator;


import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.NTProfiledPIDF;

public class ElevatorIOSim implements ElevatorIO {

    private ElevatorSim elevator;
    private NTProfiledPIDF elevatorController;



    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.ElevatorConstants.maxVelocity,
            Constants.ElevatorConstants.maxAcceleration
        );

    public ElevatorIOSim(){

        elevatorController = new NTProfiledPIDF(
            "ElevatorSim",
            Constants.ElevatorConstants.kpElevatorSim,
            Constants.ElevatorConstants.kiElevatorSim,
            Constants.ElevatorConstants.kdElevatorSim,
            Constants.ElevatorConstants.ksFeedforwardSim,
            Constants.ElevatorConstants.kvFeedforwardSim,
            constraints);


        LinearSystem<N2, N1, N2> elevatorSystem = LinearSystemId.createElevatorSystem(
            Constants.ElevatorConstants.elevatorGearbox,
            15,
            0.07,
            10
        );
        elevator = new ElevatorSim(
            elevatorSystem,
            Constants.ElevatorConstants.elevatorGearbox,
            Constants.ElevatorConstants.minHeight,
            Constants.ElevatorConstants.maxHeight,
            true,
            Constants.ElevatorConstants.minHeight
        );
    }



    @Override
    public void ioPeriodic() {

        double voltage = elevatorController.calculate(elevator.getPositionMeters());
        double gravityFeedforward = 1;

        elevator.setInputVoltage(voltage + gravityFeedforward);

        elevator.update(Constants.WristConstants.kDt);

    }



    @Override
    public void setTargetAngle(double _positionRad) {
        elevatorController.setGoal(new State(_positionRad, 0));
    }



    @Override
    public boolean hasReachedGoal() {
        return elevatorController.atGoal();
    }



    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        inputs.elevatorPosition = elevator.getPositionMeters();
        inputs.elevatorSetpointPosition = elevatorController.getSetpoint().position;
    }
}
