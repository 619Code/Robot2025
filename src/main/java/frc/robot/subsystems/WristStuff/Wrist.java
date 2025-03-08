package frc.robot.subsystems.WristStuff;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.IProfiledReset;
import frc.robot.util.Help;
import frc.robot.util.NTProfiledPIDF;

public class Wrist extends SubsystemBase implements IProfiledReset {

    private WristIO wristIO;

    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();


     private final NTProfiledPIDF controller;

    //   Network tables







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

        wristIO.updateInputs(inputs);

        setTargetAngle(getPosition());

    }

    @Override
    public void periodic() {
        if(Constants.currentMode == Mode.REPLAY){
            Logger.processInputs("RealOutputs/Wrist", inputs);
            wristIO.updateInputs(inputs);
        }else{
            wristIO.updateInputs(inputs);
            Logger.processInputs("RealOutputs/Wrist", inputs);
        }


        double voltage = controller.calculate(getPosition());

        inputs.wristSetpointPositionRad = controller.getSetpoint().position;

        double gravityFeedforward = 0.4 * Math.cos(controller.getSetpoint().position + Math.PI);


        voltage += gravityFeedforward;

        voltage = Help.clamp(voltage, -Constants.WristConstants.maxVoltage, Constants.WristConstants.maxVoltage);

        wristIO.setVoltage(voltage);

    }


    public double getPosition() {
        return inputs.wristPosition;
    }
    public double getVelocity() {
        return inputs.wristVelocity;
    }

    public void setTargetAngle(double _angleRad){
        inputs.wristGoalPositionRad = _angleRad;
        controller.setGoal(new State(_angleRad, 0));
    }


    public boolean hasReachedGoal(){
        return controller.atGoal();
    }

    @Override
    public void ProfileReset() {
        controller.setGoal(new State(getPosition(), 0));
    }
}
