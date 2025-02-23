package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    
    public Elevator(ElevatorIO _io){
        io = _io;
    }
}
