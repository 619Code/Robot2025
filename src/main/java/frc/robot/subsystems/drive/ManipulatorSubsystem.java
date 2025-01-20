package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import org.littletonrobotics.junction.Logger;
//import frc.robot.helpers.Crashboard;

public class ManipulatorSubsystem extends SubsystemBase {

    public final SparkMax intakeLeader;
    public final SparkMax shooterLeader;

    public final DigitalInput intakeProximitySensor;

    public final RelativeEncoder shooterEncoder;

    private PIDController shooterOnboardPID;
    private SimpleMotorFeedforward shooterFeedforward;

    private double shooterVelocity;


    public ManipulatorSubsystem() {

        intakeLeader = new SparkMax(Constants.ManipulatorConstants.kIntakeLeaderPort, MotorType.kBrushless);
        shooterLeader = new SparkMax(Constants.ManipulatorConstants.kShooterLeaderPort, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.inverted(Constants.ManipulatorConstants.kInakeLeaderInverted)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(35);

        intakeLeader.configure(intakeConfig, null, null);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();

        shooterConfig.inverted(Constants.ManipulatorConstants.kShooterLeaderInverted)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(35);

        shooterLeader.configure(shooterConfig, null, null);

        //intakeLeader.restoreFactoryDefaults();
        // intakeLeader.setIdleMode(IdleMode.kBrake);
        // intakeLeader.setSmartCurrentLimit(35);
        // intakeLeader.setInverted(Constants.ManipulatorConstants.kInakeLeaderInverted);

        
        //shooterLeader.restoreFactoryDefaults();
        // shooterLeader.setIdleMode(IdleMode.kBrake);        
        // shooterLeader.setSmartCurrentLimit(35);
        // shooterLeader.setInverted(Constants.ManipulatorConstants.kShooterLeaderInverted);

        intakeProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kIntakeSensorPort);

        shooterEncoder = this.shooterLeader.getEncoder();

        this.initPIDs();

    }

    public void setShooterSpeedByRPM(double speed) {
        speed = speed/60.0;
        shooterLeader.setVoltage(shooterOnboardPID.calculate(speed) + shooterFeedforward.calculate(speed));
        shooterVelocity = shooterEncoder.getVelocity();
    }

    public double getShooterRPM() {
        shooterVelocity = shooterEncoder.getVelocity();
        return shooterVelocity;
    }

    public void initPIDs() {
        shooterOnboardPID = new PIDController(Constants.ManipulatorConstants.SHOOTER_KP, Constants.ManipulatorConstants.SHOOTER_KI, Constants.ManipulatorConstants.SHOOTER_KD);
        shooterFeedforward = new SimpleMotorFeedforward(Constants.ManipulatorConstants.SHOOTER_KS, Constants.ManipulatorConstants.SHOOTER_KV, Constants.ManipulatorConstants.SHOOTER_KA);
    }

    @Override
    public void periodic() {
        
        //Crashboard.toDashboard("Sensor value: ", intakeProximitySensor.get(), "Manipulator");
        Logger.recordOutput("Sensor value: ", intakeProximitySensor.get());
        OurRobotState.hasNote = !intakeProximitySensor.get();
        

    }

    public double GetShooterVelocity(){

        return shooterEncoder.getVelocity();

    }

    public void spintake(double speed) {
        intakeLeader.set(speed);
    }

    public void spintakeVoltage(double voltage) {
        intakeLeader.setVoltage(voltage);
    }

    public void spinShooterVoltage(double voltage) {
        shooterLeader.setVoltage(voltage);
    }

    public void spinShooter(double speed) {
        shooterLeader.set(speed);
    }

    public boolean intakeTrigged() {
        return !intakeProximitySensor.get();
    }

    public void stopIntake(){
        intakeLeader.stopMotor();
    }

    public void stopShooter(){
        shooterLeader.stopMotor();
    }

    public void stopAll(){
        intakeLeader.stopMotor();
        shooterLeader.stopMotor();
    }  
}
