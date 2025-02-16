package frc.robot.subsystems.WristStuff;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NTProfiledFlex;

public class WristIOReal extends SubsystemBase implements WristIO {

    private final NTProfiledFlex wristFlex;


    private final TrapezoidProfile.State passthroughState = new TrapezoidProfile.State(Constants.WristConstants.passthroughPositionRad, 0);
    private final TrapezoidProfile.State L1State = new TrapezoidProfile.State(Constants.WristConstants.L1PositionRad, 0);
    private final TrapezoidProfile.State L2L3State = new TrapezoidProfile.State(Constants.WristConstants.L2L3PositionRad, 0);
    private final TrapezoidProfile.State L4State = new TrapezoidProfile.State(Constants.WristConstants.L4PositionRad, 0);

    private final AbsoluteEncoder wristEncoder;

    public WristIOReal(int wristMotorID) {

        //  This should be able to be default. PID values get set in the constructor fo NTProfiledFlex
        SparkFlexConfig config = new SparkFlexConfig();

        //  Set constrains
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                Constants.WristConstants.maxVelocity,
                Constants.WristConstants.maxAcceleration);

        //  Create motor
        wristFlex = new NTProfiledFlex(
            "WristFlex",
            wristMotorID,
            Constants.WristConstants.kpWrist,
            Constants.WristConstants.kiWrist,
            Constants.WristConstants.kdWrist,
            config,
            constraints
        );

        wristEncoder = wristFlex.getAbsoluteEncoder();

    }


    @Override
    public void periodic() {

        wristFlex.update();

    }

    @Override
    public void goToPassthroughAngle(){
        wristFlex.goToState(passthroughState);
    }

    @Override
    public void goToL1Angle() {
        wristFlex.goToState(L1State);
    }

    @Override
    public void goToL2L3Angle(){
        wristFlex.goToState(L2L3State);
    }

    @Override
    public void goToL4Angle(){
        wristFlex.goToState(L4State);
    }

    @Override
    public boolean hasReachedGoal(){
         return wristFlex.hasReachedGoal();
    }


    @Override
    public void updateInputs(WristIOInputs inputs){
        ifOk(wristFlex.getMotor(), wristEncoder::getPosition, (value) -> inputs.wristPosition = value);
    }
}
