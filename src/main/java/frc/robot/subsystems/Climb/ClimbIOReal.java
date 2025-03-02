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

    // Climb Inputs
    
    public static class ClimbIOInputs{
        public double position = 0.0;
        public double setpointPosition = 0.0;
    }

    // Initialization of Real Motor

    public ClimbIOReal(int climbMotorID){
        climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Update System Inputs

    public void updateInputs(ClimbIO.ClimbIOInputs inputs){
        inputs.ClimbPosition = climbEncoder.getPosition();
    }

    // Set voltage

    public void ioPeriodic(double voltage){
        climbMotor.setVoltage(voltage);
    }
}
