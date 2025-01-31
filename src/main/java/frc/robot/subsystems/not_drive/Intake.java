package frc.robot.subsystems.not_drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final SparkMax intakeMotor1;
    private final SparkMax intakeMotor2;
    private final SparkMax intakeMotor3;

    public Intake(int intakeMotorID_1, int intakeMotorID_2, int intakeMotorID_3){
        
        intakeMotor1 = new SparkMax(intakeMotorID_1, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(intakeMotorID_2, MotorType.kBrushless);
        intakeMotor3 = new SparkMax(intakeMotorID_3, MotorType.kBrushless);

        SparkMaxConfig config_1 = new SparkMaxConfig();
        config_1.idleMode(IdleMode.kBrake);

        SparkMaxConfig config_2 = new SparkMaxConfig();
        config_2.idleMode(IdleMode.kBrake);
        config_2.follow(intakeMotor1, false);

        SparkMaxConfig config_3 = new SparkMaxConfig();
        config_3.idleMode(IdleMode.kBrake);
        config_3.follow(intakeMotor1, false);

        intakeMotor1.configure(config_1, null, null);
        intakeMotor2.configure(config_2, null, null);
        intakeMotor3.configure(config_3, null, null);
    }
    // Add sensors
}
