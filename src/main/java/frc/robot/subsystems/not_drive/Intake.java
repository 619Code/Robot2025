package frc.robot.subsystems.not_drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private final SparkMax intakeMotor1;
    private final SparkMax intakeMotor2;
    private final SparkMax intakeExtensionMotor;
    
    private final RelativeEncoder extensionEncoder;
    
    private final PIDController extensionPID;

    private boolean seeking = false;

    DoubleEntry kpextensionPIDEntry;
    DoubleEntry kiextensionPIDEntry;
    DoubleEntry kdextensionPIDEntry;

    public Intake(int intakeMotorID_1, int intakeMotorID_2, int intakeExtensionMotorID){
        
        intakeMotor1 = new SparkMax(intakeMotorID_1, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(intakeMotorID_2, MotorType.kBrushless);
        intakeExtensionMotor = new SparkMax(intakeExtensionMotorID, MotorType.kBrushless);
       
        extensionEncoder = intakeExtensionMotor.getEncoder();

        SparkMaxConfig config_1 = new SparkMaxConfig();
        config_1.idleMode(IdleMode.kBrake);

        SparkMaxConfig config_2 = new SparkMaxConfig();
        config_2.idleMode(IdleMode.kBrake);
        config_2.follow(intakeMotor1, false);

        SparkMaxConfig config_3 = new SparkMaxConfig();
        config_3.idleMode(IdleMode.kBrake);

        SoftLimitConfig limitConfig = new SoftLimitConfig();
        limitConfig.forwardSoftLimit(Constants.IntakeConstants.intakeSoftUpperBound);
        limitConfig.reverseSoftLimit(Constants.IntakeConstants.intakeSoftLowerBound);

        config_3.softLimit.apply(limitConfig);

        intakeMotor1.configure(config_1, null, null);
        intakeMotor2.configure(config_2, null, null);
        intakeExtensionMotor.configure(config_3, null, null);

        extensionPID = new PIDController(0.0, 0.0, 0.0);
        
        kpextensionPIDEntry = NetworkTableInstance.getDefault().getDoubleTopic("IntakeKp").getEntry(0.0);
        kiextensionPIDEntry = NetworkTableInstance.getDefault().getDoubleTopic("IntakeKi").getEntry(0.0);
        kdextensionPIDEntry = NetworkTableInstance.getDefault().getDoubleTopic("IntakeKd").getEntry(0.0);
    }
    @Override
    public void periodic(){
        
        extensionPID.setP(kpextensionPIDEntry.get());
        extensionPID.setI(kiextensionPIDEntry.get());
        extensionPID.setD(kdextensionPIDEntry.get());

        if (!seeking) return;

        if (shouldStopSeeking()){
            stopSeeking();
            return;
        }
        double voltage = extensionPID.calculate(extensionEncoder.getPosition());
        voltage = Math.min(Math.max(voltage, -12.0), 12.0);

        intakeExtensionMotor.setVoltage(voltage);
    }
    
    private boolean shouldStopSeeking(){
        return Math.abs(extensionPID.getSetpoint() - extensionEncoder.getPosition())
            <= Constants.IntakeConstants.extensionTolerance;
    }
    
    private void stopSeeking(){
        seeking = false;
        intakeExtensionMotor.setVoltage(0.0);
    }

    private void startSeeking(){
        seeking = true;
    }

    public void goToExtendedPosition(){
        extensionPID.setSetpoint(Constants.IntakeConstants.extendedPosition);
        startSeeking();
    }

    public void goToRetractedPosition() {
        extensionPID.setSetpoint(Constants.IntakeConstants.retractedPosition);
        startSeeking();
    }
}
