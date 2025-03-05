package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.util.Help;
import frc.robot.util.NTProfiledPIDF;

public class Climb extends SubsystemBase{

    private ClimbIO climbIO;

    private final NTProfiledPIDF climbPID;

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(){
        if(Robot.isReal()){
            climbIO = new ClimbIOReal(Constants.ClimbConstants.motorId);
        }
        else{
            climbIO = new ClimbIOSim();
        }

        // Motor Initialization and Configuration

        //  Set constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                Constants.ClimbConstants.maxVelocity,
                Constants.ClimbConstants.maxAcceleration);

        climbPID = new NTProfiledPIDF(
            "Climb",
            Constants.ClimbConstants.kpWrist,
            Constants.ClimbConstants.kiWrist,
            Constants.ClimbConstants.kiWrist,
            Constants.ClimbConstants.ksFeedforward,
            Constants.ClimbConstants.kvFeedforward,
            constraints);


        //  Calling this here so that we have a value for the initial setGoal
        climbIO.updateInputs(inputs);
            //  Possibly need to change this later. Don't know if the climb will always start where we want it.
        climbPID.setGoal(new State(inputs.climbPosition, 0));

    }

    @Override
    public void periodic(){
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Climb", inputs);
            climbIO.updateInputs(inputs);
        }else{
            climbIO.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Climb", inputs);
        }

        double voltage = climbPID.calculate(inputs.climbPosition);
        voltage = Help.clamp(voltage, -Constants.ClimbConstants.maxVoltage, Constants.ClimbConstants.maxVoltage);

        climbIO.setVoltage(voltage);

    }

    // Method for specific position

    public void goToPosition(double position){
        climbPID.setGoal(new State(position, 0));
    }

    public void goToClimbOut(){
        goToPosition(Constants.ClimbConstants.climbOutPosition);
    }

    public void goToClimbIn(){
        goToPosition(Constants.ClimbConstants.climbInPosition);
    }

}
