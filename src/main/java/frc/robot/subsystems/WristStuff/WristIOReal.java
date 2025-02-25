package frc.robot.subsystems.WristStuff;

import static frc.robot.util.SparkUtil.ifOk;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.NTProfiledPIDF;

public class WristIOReal implements WristIO {

    private final SparkFlex wristFlex;
    private final NTProfiledPIDF controller;
 //   private final ArmFeedforward deviousClown;

    private final AbsoluteEncoder wristEncoder;


    private final DoublePublisher currentGoalPosPub;
    private final DoublePublisher currentVelocity;
    private final DoublePublisher desiredVelocity;
    private final DoublePublisher desiredPosition;
    private final DoublePublisher currentVoltage;


    private final DoublePublisher feedforwardVoltage;
    private final DoublePublisher pidVoltage;
    private final DoublePublisher positionalError;

    private final BooleanPublisher voltageClamped;


    public WristIOReal() {

        //  This should be able to be default. PID values get set in the constructor fo NTProfiledFlex
        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        config.absoluteEncoder.positionConversionFactor(2.0 * Math.PI);
        config.absoluteEncoder.zeroOffset(Constants.WristConstants.zeroOffset);
        config.absoluteEncoder.inverted(true);

        config.smartCurrentLimit(60);
        config.inverted(true);


  //      config.externalEncoder.measurementPeriod(1);

        config.idleMode(IdleMode.kCoast);


        SoftLimitConfig softLimits = new SoftLimitConfig();
        softLimits.forwardSoftLimit(Constants.WristConstants.softUpperLimitRadians);
        softLimits.reverseSoftLimit(Constants.WristConstants.softLowerLimitRadians);
        softLimits.forwardSoftLimitEnabled(true);
        softLimits.reverseSoftLimitEnabled(true);
        config.softLimit.apply(softLimits);


        //  These both set periodic status frame 5, but velocity should set periodic status frame 6
        config.signals.absoluteEncoderPositionPeriodMs(20);
        config.signals.absoluteEncoderVelocityPeriodMs(20);
        // config.signals.analogPositionPeriodMs(3);
        // config.signals.analogVelocityPeriodMs(4);
        // config.signals.analogVoltagePeriodMs(5);
        // config.signals.appliedOutputPeriodMs(6);
        // config.signals.busVoltagePeriodMs(7);
        // config.signals.externalOrAltEncoderPosition(8);
        // config.signals.externalOrAltEncoderVelocity(9);
        // config.signals.faultsPeriodMs(10);
        // config.signals.iAccumulationPeriodMs(11);
        // config.signals.limitsPeriodMs(12);
        // config.signals.motorTemperaturePeriodMs(13);
        // config.signals.outputCurrentPeriodMs(14);
        // config.signals.primaryEncoderPositionPeriodMs(15);
        // config.signals.primaryEncoderVelocityPeriodMs(16);
        // config.signals.warningsPeriodMs(17);


        //  Set constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                Constants.WristConstants.maxVelocity,
                Constants.WristConstants.maxAcceleration);

        //  Create motor
        wristFlex = new SparkFlex(
            Constants.WristConstants.wristMotorID,
            MotorType.kBrushless
        );



        wristFlex.configure(config, null, PersistMode.kPersistParameters);



        controller = new NTProfiledPIDF(
            "Wrist",
            Constants.WristConstants.kpWrist,
            Constants.WristConstants.kiWrist,
            Constants.WristConstants.kiWrist,
            Constants.WristConstants.ksFeedforward,
            Constants.WristConstants.kvFeedforward,
            constraints);


        wristEncoder = wristFlex.getAbsoluteEncoder();

        controller.setGoal(new State(wristEncoder.getPosition(), 0));


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


    @Override
    public void ioPeriodic() {

        double voltage = controller.calculate(wristEncoder.getPosition());
        double gravityFeedforward = 0.4 * Math.cos(controller.getSetpoint().position + Math.PI);

        //  Logging
        feedforwardVoltage.set(gravityFeedforward);
        pidVoltage.set(voltage);
        //

        voltage += gravityFeedforward;

        voltageClamped.set(Math.abs(voltage) >= Constants.WristConstants.maxVoltage);

        voltage = Math.min(voltage, Constants.WristConstants.maxVoltage);
        voltage = Math.max(voltage, -Constants.WristConstants.maxVoltage);

        wristFlex.setVoltage(voltage);


        currentVelocity.set(wristEncoder.getVelocity());
        desiredVelocity.set(controller.getSetpoint().velocity);
        desiredPosition.set(controller.getSetpoint().position);
        positionalError.set(controller.getSetpoint().position - wristEncoder.getPosition());
        currentVoltage.set(voltage);

      //  The encoder value should natively be in radians now
  //    System.out.println("Wrist encoder position radians: " + wristEncoder.getPosition());

    }

    @Override
    public void setTargetAngle(double _angleRad){
        controller.setGoal(new State(_angleRad, 0));
        currentGoalPosPub.set(_angleRad);
    }

    @Override
    public boolean hasReachedGoal(){
         return controller.atGoal();
    }


    @Override
    public void updateInputs(WristIOInputs inputs){
        ifOk(wristFlex, wristEncoder::getPosition, (value) -> inputs.wristPosition = value);
    }
}