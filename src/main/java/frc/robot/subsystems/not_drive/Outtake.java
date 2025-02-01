package frc.robot.subsystems.not_drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Outtake extends SubsystemBase{
    
    public final SparkMax outMax;
    public final SparkMax wristMax;
    public final SparkMax dislodgeMax;

    public final SparkMaxConfig wristConfig;

    public final PIDController wristPIDController;

    public final DigitalInput intakeProximitySensor;

    public Outtake(int outtakeMotorID, int wristMotorID, int dislodgerMotorID, int wristEncoderID) {

        outMax = new SparkMax(outtakeMotorID, MotorType.kBrushless);
        wristMax = new SparkMax(wristMotorID, MotorType.kBrushless);
        dislodgeMax = new SparkMax(dislodgerMotorID, MotorType.kBrushless);

        wristConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake);
        
        wristConfig.absoluteEncoder
        .positionConversionFactor(Constants.OuttakeConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(Constants.OuttakeConstants.turnEncoderVelocityFactor);

        wristMax.configure(wristConfig, null, null);

        wristPIDController = new PIDController(0.0, 0.0, 0.0);

        intakeProximitySensor = new DigitalInput(Constants.OuttakeConstants.kIntakeSensorPort);
        
    }

    public void setOuttakeVelocity(double speed) {

        outMax.set(speed);

    }

    public void setWristVelocity(Rotation2d rotation) {

        double setpoint = rotation.getRadians();
        double wristSpeed = wristPIDController.calculate(0, setpoint);
        wristMax.set(wristSpeed);

    }

    public void setDislodgeVelocity(double speed) {

        dislodgeMax.set(speed);

    }

    public double getWristPosition() {

        double position = wristMax.getAbsoluteEncoder().getPosition();

        return position;
    }

    public boolean hasCoral() {

        return !intakeProximitySensor.get();

    }

    public void stop() {
        outMax.stopMotor();
        wristMax.stopMotor();
        dislodgeMax.stopMotor();
    }


}
