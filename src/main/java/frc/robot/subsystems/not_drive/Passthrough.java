package frc.robot.subsystems.not_drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Passthrough extends SubsystemBase {
    
    private final SparkMax passthroughMotorL;
    private final SparkMax passthroughMotorR;

    public Passthrough(int passthroughMotorID_L, int passthroughMotorID_R){
        
        passthroughMotorL = new SparkMax(passthroughMotorID_L, MotorType.kBrushless);
        passthroughMotorR = new SparkMax(passthroughMotorID_R, MotorType.kBrushless);
    
        SparkMaxConfig config_L = new SparkMaxConfig();
        config_L.idleMode(IdleMode.kBrake);

        SparkMaxConfig config_R = new SparkMaxConfig();
        config_R.idleMode(IdleMode.kBrake);
        config_R.follow(passthroughMotorL, false);

        passthroughMotorL.configure(config_L, null, null);
        passthroughMotorR.configure(config_R, null, null);
    }

}
