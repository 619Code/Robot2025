package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeIOReal implements IntakeIO{

    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    private final SparkFlex intakeExtensionMotor;
    private final AbsoluteEncoder extensionEncoder;


    public IntakeIOReal(int intakeMotorID_1, int intakeExtensionMotorID){


        intakeMotor = new SparkMax(intakeMotorID_1, MotorType.kBrushless);

        intakeExtensionMotor = new SparkFlex(intakeExtensionMotorID, MotorType.kBrushless);

        extensionEncoder = intakeExtensionMotor.getAbsoluteEncoder();
        intakeEncoder = intakeMotor.getEncoder();


        SparkMaxConfig intakeMotor1Config = new SparkMaxConfig();
        intakeMotor1Config.idleMode(IdleMode.kCoast);


        SparkFlexConfig intakeExtensionMotorConfig = new SparkFlexConfig();
        intakeExtensionMotorConfig.idleMode(IdleMode.kBrake);

        intakeExtensionMotorConfig.closedLoop.feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        SoftLimitConfig limitConfig = new SoftLimitConfig();
        limitConfig.forwardSoftLimit(Constants.IntakeConstants.ExtensionMechanism.extensionSoftUpperBound);
        limitConfig.reverseSoftLimit(Constants.IntakeConstants.ExtensionMechanism.extensionSoftLowerBound);
        intakeExtensionMotorConfig.softLimit.apply(limitConfig);

        intakeMotor.configure(intakeMotor1Config, null, PersistMode.kPersistParameters);
        intakeExtensionMotor.configure(intakeExtensionMotorConfig, null, PersistMode.kPersistParameters);



    }

    @Override
    public void stopExtensionMotor() {
        intakeExtensionMotor.stopMotor();
    }

    @Override
    public void setExtensionMotorVoltage(double voltage){
        //intakeExtensionMotor.setVoltage(voltage);
    }

    public void setIntakeMotorVoltage(double voltage){
        //intakeMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.intakeExtensionPosition = extensionEncoder.getPosition();
        inputs.intakeMotorSpeedRadSec = intakeEncoder.getVelocity() * Math.PI * 2.0;
    }
}
