package frc.robot.subsystems.Passthrough;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

public class PassthroughIOReal implements PassthroughIO{

    private final SparkMax passthroughMotorL;
    private final SparkMax passthroughMotorR;

    public PassthroughIOReal(int passthroughMotorID_L, int passthroughMotorID_R) {
        passthroughMotorL = new SparkMax(passthroughMotorID_L, MotorType.kBrushless);
        passthroughMotorR = new SparkMax(passthroughMotorID_R, MotorType.kBrushless);

    SparkMaxConfig config_L = new SparkMaxConfig();
    config_L.idleMode(IdleMode.kBrake);

    SparkMaxConfig config_R = new SparkMaxConfig();
    config_R.idleMode(IdleMode.kBrake);
    config_R.follow(passthroughMotorL, false);

    passthroughMotorL.configure(config_L, null, PersistMode.kPersistParameters);
    passthroughMotorR.configure(config_R, null, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double voltage) {
        passthroughMotorL.setVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        passthroughMotorL.stopMotor();
    }

    

    @Override
    public void updateInputs(PassthroughIOInputsAutoLogged inputs) {
        inputs.motorVoltage = passthroughMotorL.getBusVoltage();
    }

    @Override
    public void periodic() {
        
    }
}
