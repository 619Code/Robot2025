package frc.robot.subsystems.Elevator;


import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

    private ElevatorSim elevator;

    public ElevatorIOSim(){


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
    public void runVoltage(double voltage) {

        elevator.setInputVoltage(voltage);

        elevator.update(Constants.kDt);

    }


    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        inputs.elevatorPosition = elevator.getPositionMeters();
    }
}
