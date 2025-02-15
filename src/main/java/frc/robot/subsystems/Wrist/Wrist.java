package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

    private WristIO io;

    private LoggableWristIOInputs inputs = new LoggableWristIOInputs();


    public Wrist(WristIO _io){
        io = _io;
    }


    @Override
    public void periodic() {
        
        io.updateInputs(inputs);
        
        Logger.processInputs("Wrist", inputs);

    }


    public void goToPassthroughAngle(){
        io.goToPassthroughAngle();
    }

    public void goToL1Angle(){
        io.goToL1Angle();
    }

    public void goToL2L3Angle(){
        io.goToL2L3Angle();
    }

    public void goToL4Angle(){
        io.goToL4Angle();
    }


    public boolean hasReachedGoal(){
        return io.hasReachedGoal();
    }
}
