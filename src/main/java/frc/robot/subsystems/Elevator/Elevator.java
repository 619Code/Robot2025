package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    
    public Elevator(ElevatorIO _io){
        io = _io;
    }


    public void GoToPassthroughHeight(){
        io.goToPassthrough();
    }
    public void GoToL1Height(){
        io.goToL1();
    }
    public void GoToL2Height(){
        io.goToL2();
    }
    public void GoToL3Height(){
        io.goToL3();
    }
    public void GoToL4Height(){
        io.goToL4();
    }
}
