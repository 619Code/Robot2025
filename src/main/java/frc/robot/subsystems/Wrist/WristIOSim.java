package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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




    private final TrapezoidProfile trapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Constants.WristConstants.maxVelocity,
            Constants.WristConstants.maxAcceleration));

    private TrapezoidProfile.State currentGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();


    private final TrapezoidProfile.State passthroughState = new TrapezoidProfile.State(Constants.WristConstants.passthroughPosition, 0);
    private final TrapezoidProfile.State L1State = new TrapezoidProfile.State(Constants.WristConstants.L1Position, 0);
    private final TrapezoidProfile.State L2L3State = new TrapezoidProfile.State(Constants.WristConstants.L2L3Position, 0);
    private final TrapezoidProfile.State L4State = new TrapezoidProfile.State(Constants.WristConstants.L4Position, 0);



    public WristIOSim(){

        wristController = new ProfiledPIDController(
            Constants.WristConstants.kpWristSim,
            Constants.WristConstants.kiWristSim,
            Constants.WristConstants.kdWristSim,
            null);

    }


    @Override
    public void periodic() {

        currentSetpoint = trapezoidProfile.calculate(Constants.WristConstants.kDt, currentSetpoint, currentGoal);

        wristController.setGoal(currentSetpoint);
        double voltage = wristController.calculate(wristMotor.getAngularPositionRad());

        //  NOTICE: CONTINUE HERE


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
}
