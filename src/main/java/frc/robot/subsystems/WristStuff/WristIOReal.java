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

import frc.robot.Constants;

public class WristIOReal implements WristIO {

    private final SparkFlex wristFlex;
 //   private final ArmFeedforward deviousClown;

    private final AbsoluteEncoder wristEncoder;


    public WristIOReal(int _wristMotorId) {

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

        //  Create motor
        wristFlex = new SparkFlex(
            _wristMotorId,
            MotorType.kBrushless
        );



        wristFlex.configure(config, null, PersistMode.kPersistParameters);

        wristEncoder = wristFlex.getAbsoluteEncoder();

    }

    @Override
    public void setVoltage(double voltage){
        wristFlex.setVoltage(voltage);
    }


    @Override
    public void updateInputs(WristIOInputs inputs){
        ifOk(wristFlex, wristEncoder::getPosition, (value) -> inputs.wristPosition = value);
        ifOk(wristFlex, wristEncoder::getVelocity, (value) -> inputs.wristVelocity = value);
    }
}
