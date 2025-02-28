package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.NTProfiledPIDF;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    // private NTProfiledPIDF elevatorController;


    // private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
    //         Constants.ElevatorConstants.maxVelocity,
    //         Constants.ElevatorConstants.maxAcceleration
    //     );

    public Elevator(ElevatorIO _io){
        io = _io;

        // elevatorController = new NTProfiledPIDF(
        //     "ElevatorSim",
        //     Constants.ElevatorConstants.kpElevatorSim,
        //     Constants.ElevatorConstants.kiElevatorSim,
        //     Constants.ElevatorConstants.kdElevatorSim,
        //     Constants.ElevatorConstants.ksFeedforwardSim,
        //     Constants.ElevatorConstants.kvFeedforwardSim,
        //     constraints);
    }

    public void periodic(){
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Elevator", inputs);
            io.updateInputs(inputs);
        }else{
            io.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Elevator", inputs);
        }

        // double voltage = elevatorController.calculate(inputs.elevatorPosition);
        // double gravityFeedforward = 0.0;  //  PUT A VALUE IN HERE


        // voltage += gravityFeedforward;

        // voltage = Math.min(voltage, Constants.ElevatorConstants.maxVoltage);
        // voltage = Math.max(voltage, -Constants.ElevatorConstants.maxVoltage);


        io.ioPeriodic();
    }

    public void setTargetPosition(ElevatorHeight _height){
        io.setTargetAngle(_height);
    }

    public boolean hasReachedGoal(){
        return io.hasReachedGoal();
    }
}
