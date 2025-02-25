package frc.robot.subsystems.WristStuff;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.NTProfiledPIDF;

public class WristIOSim implements WristIO {


    private DCMotorSim wristMotor;
    private NTProfiledPIDF wristController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.WristConstants.ksFeedforwardSim,
        Constants.WristConstants.kvFeedforwardSim);



    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.WristConstants.maxVelocity,
            Constants.WristConstants.maxAcceleration
        );


    public WristIOSim(){

        wristMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Constants.WristConstants.wristGearbox,
                0.025,
                Constants.WristConstants.wristMotorReduction),
            Constants.WristConstants.wristGearbox);


        wristController = new NTProfiledPIDF(
            "Wrist",
            Constants.WristConstants.kpWristSim,
            Constants.WristConstants.kiWristSim,
            Constants.WristConstants.kdWristSim,
            Constants.WristConstants.ksFeedforwardSim,
            Constants.WristConstants.kvFeedforwardSim,
            constraints);

    }


    @Override
    public void ioPeriodic() {

        double voltage = wristController.calculate(wristMotor.getAngularPositionRad());

        double feedforwardVoltMaybe = feedforward.calculate(wristController.getGoal().velocity);

        wristMotor.setInputVoltage(voltage + feedforwardVoltMaybe);

        wristMotor.update(Constants.WristConstants.kDt);

    }



   private void goToState(TrapezoidProfile.State _state){
        wristController.setGoal(_state);
    }

    @Override
    public void setTargetAngle(double _angleRad){
        goToState(new State(_angleRad, 0));
    }

    @Override
    public boolean hasReachedGoal(){
         return wristController.atGoal();
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristPosition = wristMotor.getAngularPositionRad();
        inputs.wristSetpointPosition = wristController.getSetpoint().position;
    }
}
