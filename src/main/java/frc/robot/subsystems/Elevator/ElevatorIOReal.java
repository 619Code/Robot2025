package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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


        leftMotorLeader.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       rightMotorFollower.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = leftMotorLeader.getEncoder();

    }



    @Override
    public void runVoltage(double voltage) {

        leftMotorLeader.setVoltage(voltage);

    }




    private SparkFlexConfig createLeftMotorConfig(){

        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder);

        config.externalEncoder.positionConversionFactor(1.0);
        //config.externalEncoder.zeroOffset(Constants.ElevatorConstants.encoderZeroOffsetRotations);
        config.externalEncoder.inverted(false);

        config.smartCurrentLimit(60);
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);


        SoftLimitConfig softLimits = new SoftLimitConfig();
        softLimits.forwardSoftLimit(Constants.ElevatorConstants.maxHeightEncoderVal);
        softLimits.reverseSoftLimit(Constants.ElevatorConstants.minHeightEncoderVal);
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
        config.follow(leftMotorLeader, true);

        return config;

    }


    private double encoderValToHeightMeters(double encoderVal){
        return (encoderVal / Constants.ElevatorConstants.maxHeightEncoderVal) *
        (Constants.ElevatorConstants.maxHeightMeters - Constants.ElevatorConstants.minHeightMeters)
         + Constants.ElevatorConstants.minHeightMeters;
    }

    private double encoderVelToVelocityMetersPerSec(double encoderVel){
        return (encoderVel / Constants.ElevatorConstants.maxHeightEncoderVal) *
        (Constants.ElevatorConstants.maxHeightMeters - Constants.ElevatorConstants.minHeightMeters);
    }



    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        inputs.elevatorPositionRotations = elevatorEncoder.getPosition();
        inputs.elevatorVelocityMPS = encoderVelToVelocityMetersPerSec(elevatorEncoder.getVelocity());

        inputs.elevatorHeightMeters = encoderValToHeightMeters(inputs.elevatorPositionRotations);
    }
}
