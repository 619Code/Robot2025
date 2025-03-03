package frc.robot.subsystems.WristStuff;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class Wrist extends SubsystemBase {

    private WristIO io;

    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public Wrist(WristIO _io){
        io = _io;
    }


    public void updateTowardsCurrentGoal() {
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Wrist", inputs);
            io.updateInputs(inputs);
        }else{
            io.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Wrist", inputs);
        }
    }


    public void setTargetAngle(double _angleRad){
        io.setTargetAngle(_angleRad);
    }


    public boolean hasReachedGoal(){
        return io.hasReachedGoal();
    }
}
