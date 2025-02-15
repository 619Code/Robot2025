package frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    
    private WristIO io;


    public Wrist(WristIO _io){
        io = _io;
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
}