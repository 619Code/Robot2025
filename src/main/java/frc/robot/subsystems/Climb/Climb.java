package frc.robot.subsystems.Climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climb extends SubsystemBase{

    private final SparkMax climbMotor;
    private ClimbIO climbIO;

    private final ProfiledPIDController climbPID;

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
            climbIO = new ClimbIOSim(climbMotorID);
        }

        climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        SoftLimitConfig limitConfig = new SoftLimitConfig();
        limitConfig.forwardSoftLimit(Constants.ClimbConstants.climbSoftUpperBound);
        limitConfig.reverseSoftLimit(Constants.ClimbConstants.climbSoftLowerBound);
        config.softLimit.apply(limitConfig);

        climbMotor.configure(config, null, null);

        kpClimbEntry = NetworkTableInstance.getDefault().getDoubleTopic("ClimbKp").getEntry(0.0);
        kiClimbEntry = NetworkTableInstance.getDefault().getDoubleTopic("ClimbKi").getEntry(0.0);
        kdClimbEntry = NetworkTableInstance.getDefault().getDoubleTopic("ClimbKd").getEntry(0.0);
        climbTargetPositionEntry = NetworkTableInstance.getDefault().getDoubleTopic("climbTargetPosition").getEntry(0);
        climbMeasured = NetworkTableInstance.getDefault().getDoubleTopic("climbMeasured").publish();
        climbVoltage = NetworkTableInstance.getDefault().getDoubleTopic("climbVoltage").publish();

        maxVelocityConstraint = NetworkTableInstance.getDefault().getDoubleTopic("climbMaxVelocity").getEntry(2.0);
        maxAccConstraint = NetworkTableInstance.getDefault().getDoubleTopic("climbMaxAcc").getEntry(1.0);

        climbPID = new ProfiledPIDController(kpClimbEntry.get(), kiClimbEntry.get(), kdClimbEntry.get(),
        new TrapezoidProfile.Constraints(maxVelocityConstraint.get(), maxAccConstraint.get()));

        kpClimbEntry.set(0);
        kiClimbEntry.set(0);
        kdClimbEntry.set(0);

        climbTargetPositionEntry.set(Constants.ClimbConstants.climbInPosition);
        maxVelocityConstraint.set(2);
        maxAccConstraint.set(1);
    }

    @Override
    public void periodic(){
        climbPID.setP(kpClimbEntry.get());
    climbPID.setI(kiClimbEntry.get());
    climbPID.setD(kdClimbEntry.get());
    climbPID.setConstraints(new TrapezoidProfile.Constraints(maxVelocityConstraint.get(), maxAccConstraint.get()));

    double pidVoltage = climbPID.calculate(climbIO.getPosition(), climbTargetPositionEntry.get());
    pidVoltage = Math.min(Math.max(pidVoltage, -12.0), 12.0);

    climbMeasured.set(climbIO.getPosition());
    climbVoltage.set(pidVoltage);
    }

    public void goToPosition(double position){
        climbTargetPositionEntry.set(position);
    }

    public void goToClimbOut(){
        goToPosition(100);
    }

    public void goToClimbIn(){
        goToPosition(0);
    }

}
