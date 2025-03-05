package frc.robot.subsystems.WristStuff;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.util.Help;
import frc.robot.util.NTProfiledPIDF;

public class Wrist extends SubsystemBase {

    private WristIO wristIO;

    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();


     private final NTProfiledPIDF controller;

    //   Network tables

    private final DoublePublisher currentGoalPosPub;
    private final DoublePublisher currentVelocity;
    private final DoublePublisher desiredVelocity;
    private final DoublePublisher desiredPosition;
    private final DoublePublisher currentVoltage;


    private final DoublePublisher feedforwardVoltage;
    private final DoublePublisher pidVoltage;
    private final DoublePublisher positionalError;

    private final BooleanPublisher voltageClamped;






    public Wrist(){
        if(Robot.isReal()){
            wristIO = new WristIOReal(Constants.WristConstants.wristMotorID);
        }
        else{
            wristIO = new WristIOSim();
        }


        //  Set constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                Constants.WristConstants.Constraints.maxVelocity,
                Constants.WristConstants.Constraints.maxAcceleration);

        controller = new NTProfiledPIDF(
            "Wrist",
            Constants.WristConstants.Control.kpWrist,
            Constants.WristConstants.Control.kiWrist,
            Constants.WristConstants.Control.kiWrist,
            Constants.WristConstants.Control.ksFeedforward,
            Constants.WristConstants.Control.kvFeedforward,
            constraints);

        controller.setGoal(new State(getPosition(), 0));




        currentGoalPosPub = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/goal pos").publish();
        currentVelocity = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/current velocity").publish();
        desiredVelocity = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/desired velocity").publish();
        desiredPosition = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/desired position").publish();
        currentVoltage = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/current voltage").publish();
        feedforwardVoltage = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/feedforward voltage").publish();
        pidVoltage = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/pid voltage").publish();
        positionalError = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex/position error").publish();
        voltageClamped = NetworkTableInstance.getDefault().getBooleanTopic("WristFlex/voltage clamped").publish();

        currentGoalPosPub.set(0);
        currentVelocity.set(0);
        desiredVelocity.set(0);
        desiredPosition.set(0);
        currentVoltage.set(0);
        feedforwardVoltage.set(0);
        pidVoltage.set(0);
        positionalError.set(0);
        voltageClamped.set(false);
    }


    public void updateTowardsCurrentGoal() {
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Wrist", inputs);
            wristIO.updateInputs(inputs);
        }else{
            wristIO.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Wrist", inputs);
        }


        double voltage = controller.calculate(getPosition());
        double gravityFeedforward = 0.4 * Math.cos(controller.getSetpoint().position + Math.PI);

        //  Logging
        feedforwardVoltage.set(gravityFeedforward);
        pidVoltage.set(voltage);
        //

        voltage += gravityFeedforward;

        voltageClamped.set(Math.abs(voltage) >= Constants.WristConstants.maxVoltage);

        voltage = Help.clamp(voltage, -Constants.WristConstants.maxVoltage, Constants.WristConstants.maxVoltage);

        wristIO.setVoltage(voltage);


        currentVelocity.set(getVelocity());
        desiredVelocity.set(controller.getSetpoint().velocity);
        desiredPosition.set(controller.getSetpoint().position);
        positionalError.set(controller.getSetpoint().position - getPosition());
        currentVoltage.set(voltage);
    }


    public double getPosition() {
        return inputs.wristPosition;
    }
    public double getVelocity() {
        return inputs.wristVelocity;
    }

    public void setTargetAngle(double _angleRad){
        wristIO.setVoltage(_angleRad);
    }


    public boolean hasReachedGoal(){
        return controller.atGoal();
    }
}
