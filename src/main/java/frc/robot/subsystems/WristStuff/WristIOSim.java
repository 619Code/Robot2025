package frc.robot.subsystems.WristStuff;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristIOSim extends SubsystemBase implements WristIO {


    private DCMotorSim wristMotor;
    private ProfiledPIDController wristController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.WristConstants.ksFeedforwardSim,
        Constants.WristConstants.kvFeedforwardSim);



    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.WristConstants.maxVelocity,
            Constants.WristConstants.maxAcceleration
        );

    private TrapezoidProfile.State currentGoal = new TrapezoidProfile.State();


    private final TrapezoidProfile.State passthroughState = new TrapezoidProfile.State(Constants.WristConstants.passthroughPositionRad, 0);
    private final TrapezoidProfile.State L1State = new TrapezoidProfile.State(Constants.WristConstants.L1PositionRad, 0);
    private final TrapezoidProfile.State L2L3State = new TrapezoidProfile.State(Constants.WristConstants.L2L3PositionRad, 0);
    private final TrapezoidProfile.State L4State = new TrapezoidProfile.State(Constants.WristConstants.L4PositionRad, 0);



    public WristIOSim(){

        wristMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Constants.WristConstants.wristGearbox,
                0.025,
                Constants.WristConstants.wristMotorReduction),
            Constants.WristConstants.wristGearbox);


        wristController = new ProfiledPIDController(
            Constants.WristConstants.kpWristSim,
            Constants.WristConstants.kiWristSim,
            Constants.WristConstants.kdWristSim,
            constraints);

        wristController.setConstraints(constraints);

    }


    @Override
    public void periodic() {

        double voltage = wristController.calculate(wristMotor.getAngularPositionRad(), currentGoal);

        double feedforwardVoltMaybe = feedforward.calculate(wristController.getGoal().velocity);

        wristMotor.setInputVoltage(voltage + feedforwardVoltMaybe);

        wristMotor.update(Constants.WristConstants.kDt);

    }



   private void goToState(TrapezoidProfile.State _state){
        currentGoal = _state;
    }

    @Override
    public void goToPassthroughAngle(){
        goToState(passthroughState);
    }

    @Override
    public void goToL1Angle() {
        goToState(L1State);
    }

    @Override
    public void goToL2L3Angle(){
        goToState(L2L3State);
    }

    @Override
    public void goToL4Angle(){
        goToState(L4State);
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
