package frc.robot.subsystems.Climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.NTProfiledPIDF;

public class Climb extends SubsystemBase{

    private final SparkMax climbMotor;
    private final NTProfiledPIDF climbPID;
    private ClimbIO climbIO;

    DoubleEntry kpClimbEntry;
    DoubleEntry kiClimbEntry;
    DoubleEntry kdClimbEntry;
    DoubleEntry climbTargetPositionEntry;
    DoublePublisher climbMeasured;
    DoublePublisher climbVoltage;

    DoubleEntry maxVelocityConstraint;
    DoubleEntry maxAccConstraint;

    public Climb(int climbMotorID){
        if(Robot.isReal()){
            climbIO = new ClimbIOReal(climbMotorID);
        }
        else{
            climbIO = new ClimbIOSim();
        }

        // Motor Initialization and Configuration

        climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        SoftLimitConfig limitConfig = new SoftLimitConfig();
        limitConfig.forwardSoftLimit(Constants.ClimbConstants.climbSoftUpperBound);
        limitConfig.reverseSoftLimit(Constants.ClimbConstants.climbSoftLowerBound);
        config.softLimit.apply(limitConfig);

        climbMotor.configure(config, null, null);

        // PID Constraints and Initialization

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.ClimbConstants.maxVelocity,
            Constants.ClimbConstants.maxAcceleration);

            climbPID = new NTProfiledPIDF("Intake",
            Constants.ClimbConstants.kp,
            Constants.ClimbConstants.ki,
            Constants.ClimbConstants.kd,
            Constants.ClimbConstants.ksFeedforward,
            Constants.ClimbConstants.kvFeedforward,
            constraints);

        // Initialization of NetworkTables

        climbTargetPositionEntry = NetworkTableInstance.getDefault().getDoubleTopic("climbTargetPosition").getEntry(90);
        climbMeasured = NetworkTableInstance.getDefault().getDoubleTopic("climbMeasured").getEntry(2);
        climbVoltage = NetworkTableInstance.getDefault().getDoubleTopic("climbVoltage").getEntry(1);

        maxVelocityConstraint = NetworkTableInstance.getDefault().getDoubleTopic("climbMaxVelocity").getEntry(20.0);
        maxAccConstraint = NetworkTableInstance.getDefault().getDoubleTopic("climbMaxAcc").getEntry(10.0);

        // Default Values

        climbTargetPositionEntry.set(Constants.ClimbConstants.climbInPosition);
        maxVelocityConstraint.set(20);
        maxAccConstraint.set(10);
    }

    @Override
    public void periodic(){
    
    // Get targetPosition and set PID goal
        
    double targetPosition = climbTargetPositionEntry.get();
    climbPID.setGoal(new State(targetPosition, 0));
    
    // ClimbIO Current Position

    ClimbIO.ClimbIOInputs inputs = new ClimbIO.ClimbIOInputs();
    climbIO.updateInputs(inputs);
    double currentPosition = inputs.ClimbPosition;

    // Voltage

    double voltage = climbPID.calculate(currentPosition);
    voltage = Math.min(Math.max(voltage, -12.0), 12.0);

    climbVoltage.set(voltage);
    climbIO.ioPeriodic(voltage);
    climbMeasured.set(inputs.ClimbPosition);
    }

    // Methods for specific positions

    public void goToPosition(double degrees){
        climbPID.setGoal(new State(degrees, 0));
        climbTargetPositionEntry.set(degrees);
    }

    public void goToClimbOut(){
        goToPosition(100);
    }

    public void goToClimbIn(){
        goToPosition(0);
    }

    // Check if Intake reached targetPosition

    public boolean hasReachedGoal(){
        return false;
    }

}
