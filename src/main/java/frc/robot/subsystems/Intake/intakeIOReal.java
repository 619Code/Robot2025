package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
        SparkMaxConfig intakeConfigure = new SparkMaxConfig();

        intakeConfigure.idleMode(IdleMode.kCoast);

        intakeExtensionMotor.configure(intakeConfigure, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



}
