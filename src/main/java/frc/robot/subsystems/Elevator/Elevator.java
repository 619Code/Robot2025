package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.IProfiledReset;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.util.Help;
import frc.robot.util.NTProfiledPIDF;

public class Elevator extends SubsystemBase implements IProfiledReset {

    private final ElevatorIO elevatorIO;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private NTProfiledPIDF elevatorController;

    private final DoubleEntry gravityFeedforward;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.ElevatorConstants.maxVelocityMetersPerSec,
            Constants.ElevatorConstants.maxAccelerationMetersPerSecSqrd
        );

    public Elevator(){
        if(Robot.isReal()){
            elevatorIO = new ElevatorIOReal(
                Constants.ElevatorConstants.leftMotorID,
                Constants.ElevatorConstants.rightMotorID
            );
        }
        else{
            elevatorIO = new ElevatorIOSim();
        }

        elevatorController = new NTProfiledPIDF(
            "Elevator",
            Constants.ElevatorConstants.kpElevator,
            Constants.ElevatorConstants.kiElevator,
            Constants.ElevatorConstants.kdElevator,
            Constants.ElevatorConstants.ksFeedforward,
            Constants.ElevatorConstants.kvFeedforward,
            constraints
        );


        //  This needs to be called before the line below it
        elevatorIO.updateInputs(inputs);
        elevatorController.setGoal(new State(getPositionMeters(), 0));
        inputs.elevatorGoalMeters = elevatorController.getGoal().position;


        gravityFeedforward = NetworkTableInstance.getDefault().getDoubleTopic("ElevatorGravityFF").getEntry(Constants.ElevatorConstants.feedforwardGravity);

        gravityFeedforward.set(Constants.ElevatorConstants.feedforwardGravity);

    }

    public void periodic(){
        //  Figure out a better way to do this elevatorSetpointPosition thing
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Elevator", inputs);
            elevatorIO.updateInputs(inputs);
        }else{
            elevatorIO.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Elevator", inputs);
        }

        double voltage = elevatorController.calculate(getPositionMeters());

        inputs.elevatorSetpointPositionMeters = elevatorController.getSetpoint().position;


        if(inputs.elevatorSetpointPositionMeters > Constants.ElevatorConstants.minHeightMeters + 0.02){
            voltage += gravityFeedforward.get();
        }

        voltage = Help.clamp(voltage, -Constants.ElevatorConstants.maxVoltage, Constants.ElevatorConstants.maxVoltage);


        inputs.elevatorVoltage = voltage;
        elevatorIO.runVoltage(voltage);

    }

    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }

    public void setTargetPosition(ElevatorHeight _height){
        inputs.elevatorGoalMeters = _height.heightMeters;
        inputs.elevatorGoalEnum = _height;
        elevatorController.setGoal(new State(_height.heightMeters, 0));
    }

    public void shiftTargetPosition(double amount){
        elevatorController.setGoal(new State(getPositionMeters() + (amount * 0.001), 0));
    }

    public ElevatorHeight getCurrentGoal(){
        return inputs.elevatorGoalEnum;
    }

    public boolean hasReachedGoal(){
       return elevatorController.atGoal();
    }


    @Override
    public void ProfileReset() {
        elevatorController.setGoal(new State(getPositionMeters(), 0));
    }
}
