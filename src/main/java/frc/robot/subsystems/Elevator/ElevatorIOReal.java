package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.util.NTProfiledPIDF;

public class ElevatorIOReal implements ElevatorIO {

    private final SparkFlex leftMotorLeader;
    private final SparkFlex rightMotorFollower;
    private final NTProfiledPIDF controller;


    private final AbsoluteEncoder elevatorEncoder;

    public ElevatorIOReal(){

        leftMotorLeader = new SparkFlex(Constants.ElevatorConstants.leftMotorID, MotorType.kBrushless);
        rightMotorFollower = new SparkFlex(Constants.ElevatorConstants.rightMotorID, MotorType.kBrushless);

        //  Left motor config
        SparkFlexConfig leftMotorConfig = createLeftMotorConfig();
        //  Right motor config
        SparkFlexConfig rightMotorConfig = createRightMotorConfig();


        leftMotorLeader.configure(leftMotorConfig, null, PersistMode.kPersistParameters);
        rightMotorFollower.configure(rightMotorConfig, null, PersistMode.kPersistParameters);

        elevatorEncoder = leftMotorLeader.getAbsoluteEncoder();


        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.maxVelocity,
                Constants.ElevatorConstants.maxAcceleration);

        controller = new NTProfiledPIDF(
            "Elevator",
            Constants.ElevatorConstants.kpElevator,
            Constants.ElevatorConstants.kiElevator,
            Constants.ElevatorConstants.kiElevator,
            Constants.ElevatorConstants.ksFeedforward,
            Constants.ElevatorConstants.kvFeedforward,
            constraints
        );

        controller.setGoal(new State(elevatorEncoder.getPosition(), 0));

    }



    @Override
    public void ioPeriodic() {

        double voltage = controller.calculate(elevatorEncoder.getPosition());
        double gravityFeedforward = 0.0;  //  PUT A VALUE IN HERE


        voltage += gravityFeedforward;

        voltage = Math.min(voltage, Constants.ElevatorConstants.maxVoltage);
        voltage = Math.max(voltage, -Constants.ElevatorConstants.maxVoltage);

        leftMotorLeader.setVoltage(voltage);

    }




    private SparkFlexConfig createLeftMotorConfig(){

        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        config.absoluteEncoder.positionConversionFactor(Constants.ElevatorConstants.elevatorEncoderConversionFactor);
        config.absoluteEncoder.zeroOffset(Constants.ElevatorConstants.encoderZeroOffsetRotations);
        config.absoluteEncoder.inverted(false);

        config.smartCurrentLimit(60);
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);


        SoftLimitConfig softLimits = new SoftLimitConfig();
        softLimits.forwardSoftLimit(Constants.ElevatorConstants.maxHeightMeters);
        softLimits.reverseSoftLimit(Constants.ElevatorConstants.minHeightMeters);
        softLimits.forwardSoftLimitEnabled(true);
        softLimits.reverseSoftLimitEnabled(true);
        config.softLimit.apply(softLimits);


        //  These both set periodic status frame 5, but velocity should set periodic status frame 6
        config.signals.absoluteEncoderPositionPeriodMs(20);
        config.signals.absoluteEncoderVelocityPeriodMs(20);


        return config;

    }

    private SparkFlexConfig createRightMotorConfig(){

        SparkFlexConfig config = createLeftMotorConfig();
        config.inverted(true);
        config.follow(leftMotorLeader);

        return config;

    }



    @Override
    public void setTargetAngle(ElevatorHeight _height) {
        controller.setGoal(new State(_height.heightMeters, 0));
    }



    @Override
    public boolean hasReachedGoal() {
        return controller.atGoal();
    }



    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        inputs.elevatorPosition = elevatorEncoder.getPosition();
        inputs.elevatorSetpointPosition = controller.getSetpoint().position;
    }
}
