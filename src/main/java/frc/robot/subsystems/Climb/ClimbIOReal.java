package frc.robot.subsystems.Climb;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ClimbIOReal implements ClimbIO{

    private final SparkMax climbMotor;
    private final RelativeEncoder climbEncoder;



    public ClimbIOReal(int climbMotorID){

        climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        SoftLimitConfig limitConfig = new SoftLimitConfig();
        limitConfig.forwardSoftLimit(Constants.ClimbConstants.climbSoftUpperBound);
        limitConfig.reverseSoftLimit(Constants.ClimbConstants.climbSoftLowerBound);
        config.softLimit.apply(limitConfig);

        climbMotor.configure(config, null, PersistMode.kPersistParameters);

        climbEncoder = climbMotor.getEncoder();

    }

    @Override
    public void stopMotor(){
        climbMotor.stopMotor();
    }

    @Override
    public void setVoltage(double voltage){
        climbMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ClimbIOInputsAutoLogged inputs) {
        inputs.climbPosition = climbEncoder.getPosition();
    }
}
