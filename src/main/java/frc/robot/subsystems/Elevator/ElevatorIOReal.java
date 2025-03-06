package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {

    private final SparkFlex leftMotorLeader;
    private final SparkFlex rightMotorFollower;


    private final RelativeEncoder elevatorEncoder;

    public ElevatorIOReal(int leftMotorId, int rightMotorId){

        leftMotorLeader = new SparkFlex(leftMotorId, MotorType.kBrushless);
        rightMotorFollower = new SparkFlex(rightMotorId, MotorType.kBrushless);

        //  Left motor config
        SparkFlexConfig leftMotorConfig = createLeftMotorConfig();
        //  Right motor config
        SparkFlexConfig rightMotorConfig = createRightMotorConfig();


        leftMotorLeader.configure(leftMotorConfig, null, PersistMode.kPersistParameters);
        rightMotorFollower.configure(rightMotorConfig, null, PersistMode.kPersistParameters);

        elevatorEncoder = leftMotorLeader.getEncoder();

    }



    @Override
    public void runVoltage(double voltage) {

    //    leftMotorLeader.setVoltage(voltage);
    leftMotorLeader.setVoltage(0);

    }




    private SparkFlexConfig createLeftMotorConfig(){

        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder);

        config.externalEncoder.positionConversionFactor(Constants.ElevatorConstants.elevatorEncoderConversionFactor);
        //config.externalEncoder.zeroOffset(Constants.ElevatorConstants.encoderZeroOffsetRotations);
        config.externalEncoder.inverted(false);

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
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        inputs.elevatorPosition = elevatorEncoder.getPosition();
    }
}
