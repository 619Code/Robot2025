package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeIOReal implements IntakeIO{

    private final SparkMax intakeMotor;

    private final SparkMax intakeExtensionMotor;
    private final AbsoluteEncoder extensionEncoder;


    public IntakeIOReal(int intakeMotorID_1, int intakeExtensionMotorID){


        intakeMotor = new SparkMax(intakeMotorID_1, MotorType.kBrushless);

        intakeExtensionMotor = new SparkMax(intakeExtensionMotorID, MotorType.kBrushless);

        extensionEncoder = intakeExtensionMotor.getAbsoluteEncoder();


        SparkMaxConfig intakeMotor1Config = new SparkMaxConfig();
        intakeMotor1Config.idleMode(IdleMode.kBrake);

        SparkMaxConfig config_3 = new SparkMaxConfig();
        config_3.idleMode(IdleMode.kBrake);

        SoftLimitConfig limitConfig = new SoftLimitConfig();
        limitConfig.forwardSoftLimit(Constants.IntakeConstants.intakeSoftUpperBound);
        limitConfig.reverseSoftLimit(Constants.IntakeConstants.intakeSoftLowerBound);
        config_3.softLimit.apply(limitConfig);

        intakeMotor.configure(intakeMotor1Config, null, PersistMode.kPersistParameters);
        intakeExtensionMotor.configure(config_3, null, PersistMode.kPersistParameters);



    }

    @Override
    public double getPosition() {
        return extensionEncoder.getPosition();
    }

    @Override
    public void stopMotor() {
    intakeExtensionMotor.stopMotor();
    }

    @Override
    public void setExtensionMotorVoltage(double voltage){
        intakeExtensionMotor.setVoltage(voltage);
    }

    public void setIntakeMotorVoltage(double voltage){
        intakeMotor.setVoltage(voltage);
    }

    // @Override
    // public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    //     throw new UnsupportedOperationException("NOT IMPLEMENTED");
    // }
}
