package frc.robot.subsystems.WristStuff;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

    private WristIO io;

    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public Wrist(WristIO _io){
        io = _io;
    }


    @Override
    public void periodic() {

        io.updateInputs(inputs);

        Logger.processInputs("Wrist", inputs);

    }


    public void goToPassthroughAngle(){
        System.out.println("Go to passthrough");
        io.goToPassthroughAngle();
    }

    public void goToL1Angle(){
        System.out.println("Go to TROUGH LEVEL");
        io.goToL1Angle();
    }

    public void goToL2L3Angle(){
        System.out.println("Go to middle two levels");
        io.goToL2L3Angle();
    }

    public void goToL4Angle(){
        System.out.println("Go to highest L4 thing");
        io.goToL4Angle();
    }


    public boolean hasReachedGoal(){
        return io.hasReachedGoal();
    }
}
