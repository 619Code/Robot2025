package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.NTProfiledPIDF;


public class intakeIOReal implements IntakeIO{
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final NTProfiledPIDF controller;

    private final DoublePublisher currentGoalPosPub;
    private final DoublePublisher currentVelocity;
    private final DoublePublisher desiredVelocity;
    private final DoublePublisher desiredPosition;
    private final DoublePublisher currentVoltage;
    private final DoublePublisher feedforwardVoltage;
    private final DoublePublisher pidVoltage;
    private final DoublePublisher positionalError;
    private final BooleanPublisher voltageClamped;

    public static class IntakeIOInputs {
        public double position = 0.0;
        public double setpointPosition = 0.0;
    }

    public intakeIOReal(int intakeMotorID){
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.IntakeConstants.maxVelocity,
            Constants.IntakeConstants.maxAcceleration);

        controller = new NTProfiledPIDF("Intake",
        Constants.IntakeConstants.kp,
        Constants.IntakeConstants.ki,
        Constants.IntakeConstants.kd,
        Constants.IntakeConstants.ksFeedforward,
        Constants.IntakeConstants.kvFeedforward,
        constraints);

        controller.setGoal(new State(intakeEncoder.getPosition(), 0));

        currentGoalPosPub = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/goal pos").publish();
        currentVelocity = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/current velocity").publish();
        desiredVelocity = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/desired velocity").publish();
        desiredPosition = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/desired position").publish();
        currentVoltage = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/current voltage").publish();
        feedforwardVoltage = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/feedforward voltage").publish();
        pidVoltage = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/pid voltage").publish();
        positionalError = NetworkTableInstance.getDefault()
                .getDoubleTopic("Intake/position error").publish();
        voltageClamped = NetworkTableInstance.getDefault()
                .getBooleanTopic("Intake/voltage clamped").publish();

        currentGoalPosPub.set(intakeEncoder.getPosition());
        currentVelocity.set(0.0);
        desiredVelocity.set(0.0);
        desiredPosition.set(intakeEncoder.getPosition());
        currentVoltage.set(0.0);
        feedforwardVoltage.set(0.0);
        pidVoltage.set(0.0);
        positionalError.set(0.0);
        voltageClamped.set(false);
    }

    public void setTargetPosition(double setTargetPosition){
        controller.setGoal(new State(setTargetPosition, 0));
        currentGoalPosPub.set(setTargetPosition);
    }

    public boolean hasReachedGoal(){
        return controller.atGoal();
    }

    public void updateInputs(IntakeIO.IntakeIOInputs inputs){
        inputs.intakePosition = intakeEncoder.getPosition();
        inputs.intakeSetpointPosition = controller.getSetpoint().position;
    }

    public void ioPeriodic(){
        double voltage = controller.calculate(intakeEncoder.getPosition());

        double feedforward = 0.0;
        feedforwardVoltage.set(feedforward);
        pidVoltage.set(voltage);
        voltage += feedforward;

        voltageClamped.set(Math.abs(voltage) >= Constants.IntakeConstants.maxVoltage);
        voltage = Math.min(voltage, Constants.IntakeConstants.maxVoltage);
        voltage = Math.max(voltage, -Constants.IntakeConstants.maxVoltage);
        intakeMotor.setVoltage(voltage);

        currentVelocity.set(intakeEncoder.getVelocity());
        desiredVelocity.set(controller.getSetpoint().velocity);
        desiredPosition.set(controller.getSetpoint().position);
        positionalError.set(controller.getSetpoint().position - intakeEncoder.getPosition());
        currentVoltage.set(voltage);
    }
}
