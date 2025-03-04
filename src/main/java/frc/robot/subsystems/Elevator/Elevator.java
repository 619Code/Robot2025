package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.util.NTProfiledPIDF;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private NTProfiledPIDF elevatorController;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.ElevatorConstants.maxVelocity,
            Constants.ElevatorConstants.maxAcceleration
        );

    public Elevator(ElevatorIO _io){
        io = _io;

        elevatorController = new NTProfiledPIDF(
            "Elevator",
            Constants.ElevatorConstants.kpElevator,
            Constants.ElevatorConstants.kiElevator,
            Constants.ElevatorConstants.kdElevator,
            Constants.ElevatorConstants.ksFeedforward,
            Constants.ElevatorConstants.kvFeedforward,
            constraints);

        elevatorController.setGoal(new State(getPositionMeters(), 0));
    }

    public void periodic(){
        //  Figure out a better way to do this elevatorSetpointPosition thing
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Elevator", inputs);
            io.updateInputs(inputs);
        }else{
            io.updateInputs(inputs);
            inputs.elevatorSetpointPosition = elevatorController.getSetpoint().position;
            Logger.processInputs("RealOutputs/Elevator", inputs);
        }

        double voltage = elevatorController.calculate(inputs.elevatorPosition);
        double gravityFeedforward = 0.5;  //  PUT A VALUE IN HERE


        voltage += gravityFeedforward;

        voltage = Math.min(voltage, Constants.ElevatorConstants.maxVoltage);
        voltage = Math.max(voltage, -Constants.ElevatorConstants.maxVoltage);


        io.runVoltage(voltage);

    }

    public double getPositionMeters(){
        return inputs.elevatorPosition;
    }

    public void setTargetPosition(ElevatorHeight _height){
        elevatorController.setGoal(new State(_height.heightMeters, 0));
    }

    public boolean hasReachedGoal(){
        return elevatorController.atGoal();
    }
}
