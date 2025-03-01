package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class intakeIOReal implements IntakeIO{
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    // Intake Inputs

    public static class IntakeIOInputs {
        public double position = 0.0;
        public double setpointPosition = 0.0;
    }

    // Initialization of Real Motor

    public intakeIOReal(int intakeMotorID){
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Update System Inputs

    public void updateInputs(IntakeIO.IntakeIOInputs inputs){
        inputs.intakePosition = intakeEncoder.getPosition();
    }

    // Set voltage

    public void ioPeriodic(double voltage){
        intakeMotor.setVoltage(voltage);
    }
}
