package frc.robot.subsystems.Climb;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbIOReal implements ClimbIO{

    private final SparkMax climbMotor;
    private final RelativeEncoder climbEncoder;

    @Override
    public double getPosition(){
        return climbEncoder.getPosition();
    }

    @Override
    public void stopMotor(){
        climbMotor.stopMotor();
    }

    @Override
    public void setVoltage(double voltage){
        climbMotor.setVoltage(voltage);
    }

    public ClimbIOReal(int climbMotorID){
        climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast);

        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
