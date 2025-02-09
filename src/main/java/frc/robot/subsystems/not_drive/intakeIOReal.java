package frc.robot.subsystems.not_drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class intakeIOReal implements IntakeIO{

    private final SparkMax intakeExtensionMotor;
    private final RelativeEncoder extensionEncoder;

    @Override
    public double getPosition() {
        return extensionEncoder.getPosition();
    }

    @Override
    public void stopMotor() {
    intakeExtensionMotor.stopMotor();
    }

    public void setVoltage(double voltage){
        intakeExtensionMotor.setVoltage(voltage);
    }

    @Override
    public void update() {
    }

    public intakeIOReal(int intakeExtensionMotorID){
        intakeExtensionMotor = new SparkMax(intakeExtensionMotorID, MotorType.kBrushless);
        extensionEncoder = intakeExtensionMotor.getEncoder();
        SparkMaxConfig config_3 = new SparkMaxConfig();
        intakeExtensionMotor.configure(config_3, null, null);
    }



}
