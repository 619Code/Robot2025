package frc.robot.subsystems.Elevator;


import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;

public class ElevatorIOSim implements ElevatorIO {

    private ElevatorSim elevator;
    //private NTProfiledPIDF elevatorController;



    // private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
    //         Constants.ElevatorConstants.maxVelocity,
    //         Constants.ElevatorConstants.maxAcceleration
    //     );

    public ElevatorIOSim(){

        // elevatorController = new NTProfiledPIDF(
        //     "ElevatorSim",
        //     // Constants.ElevatorConstants.kpElevatorSim,
        //     // Constants.ElevatorConstants.kiElevatorSim,
        //     // Constants.ElevatorConstants.kdElevatorSim,
        //     // Constants.ElevatorConstants.ksFeedforwardSim,
        //     // Constants.ElevatorConstants.kvFeedforwardSim,
        //     Constants.ElevatorConstants.kpElevator,
        //     Constants.ElevatorConstants.kiElevator,
        //     Constants.ElevatorConstants.kiElevator,
        //     Constants.ElevatorConstants.ksFeedforward,
        //     Constants.ElevatorConstants.kvFeedforward,
        //     constraints);


        LinearSystem<N2, N1, N2> elevatorSystem = LinearSystemId.createElevatorSystem(
            Constants.ElevatorConstants.elevatorGearbox,
            15,
            0.07,
            10
        );

        elevator = new ElevatorSim(
            elevatorSystem,
            Constants.ElevatorConstants.elevatorGearbox,
            Constants.ElevatorConstants.minHeightMeters,
            Constants.ElevatorConstants.maxHeightMeters,
            true,
            Constants.ElevatorConstants.minHeightMeters
        );
    }



    @Override
    public void ioPeriodic(double voltage) {


        // double voltage = elevatorController.calculate(elevator.getPositionMeters());
        // double gravityFeedforward = 0.0;  //  PUT A VALUE IN HERE


        // voltage += gravityFeedforward;

        // voltage = Math.min(voltage, Constants.ElevatorConstants.maxVoltage);
        // voltage = Math.max(voltage, -Constants.ElevatorConstants.maxVoltage);

        elevator.setInputVoltage(voltage);

        elevator.update(Constants.WristConstants.kDt);

    }



    @Override
    public void setTargetAngle(ElevatorHeight _height) {
        //elevatorController.setGoal(new State(_height.heightMeters, 0));
    }



    @Override
    public boolean hasReachedGoal() {
        //return elevatorController.atGoal();
        return true;
    }



    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        inputs.elevatorPosition = elevator.getPositionMeters();
        //inputs.elevatorSetpointPosition = elevatorController.getSetpoint().position;
    }
}
