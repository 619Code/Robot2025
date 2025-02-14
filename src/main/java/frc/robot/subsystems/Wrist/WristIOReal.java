package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class WristIOReal extends SubsystemBase implements WristIO {

    private final SparkFlex wristFlex;
    private final SparkClosedLoopController wristController;
    //  private final AbsoluteEncoder wristEncoder;

    // Note: These gains are fake, and will have to be tuned for your robot.
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.WristConstants.ksFeedforward, 
        Constants.WristConstants.kvFeedforward);

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint.
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


    public WristIOReal(int wristMotorID) {

        wristFlex = new SparkFlex(wristMotorID, MotorType.kBrushless);
        SparkFlexConfig wristConfig = new SparkFlexConfig();
        //  Configure
        wristConfig.closedLoop
        .p(Constants.WristConstants.kpWrist)
        .i(Constants.WristConstants.kiWrist)
        .d(Constants.WristConstants.kdWrist);
        wristFlex.configure(wristConfig, null, null);


        wristController = wristFlex.getClosedLoopController();

    //      wristEncoder = wristMax.getAbsoluteEncoder();
        
    }


    @Override
    public void periodic() {

        currentSetpoint = trapezoidProfile.calculate(Constants.WristConstants.kDt, currentSetpoint, currentGoal);

        wristController.setReference(
            currentSetpoint.position, 
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, 
            feedforward.calculate(currentSetpoint.velocity) / 12.0);

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

