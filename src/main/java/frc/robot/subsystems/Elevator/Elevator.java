package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO _io){
        io = _io;
    }


    public void updateTowardsCurrentGoal(){
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Elevator", inputs);
            io.updateInputs(inputs);
        }else{
            io.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Elevator", inputs);
        }

        io.ioPeriodic();
    }

    public void setTargetPosition(double _positionRad){
        io.setTargetAngle(_positionRad);
    }

    public boolean hasReachedGoal(){
        return io.hasReachedGoal();
    }
}
