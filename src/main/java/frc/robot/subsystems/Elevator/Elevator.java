package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO _io){
        io = _io;
    }


    public void periodic(){
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Elevator", inputs);
            io.updateInputs(inputs);
        }else{
            io.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Elevator", inputs);
        }
    }

    public void setTargetPosition(ElevatorHeight _height){
        io.setTargetAngle(_height);
    }

    public boolean hasReachedGoal(){
        return io.hasReachedGoal();
    }
}
