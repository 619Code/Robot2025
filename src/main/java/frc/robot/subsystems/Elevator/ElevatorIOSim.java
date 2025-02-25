package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.NTProfiledPIDF;

public class ElevatorIOSim implements ElevatorIO {

    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void ioPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'ioPeriodic'");
    }

    @Override
    public void setTargetAngle(double _positionRad) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetAngle'");
    }

    @Override
    public boolean hasReachedGoal() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasReachedGoal'");
    }

    // private DCMotorSim leftMotor;
    // private DCMotorSim rightMotor;
    // private NTProfiledPIDF leftController;


    // private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    //     Constants.ElevatorConstants.ksFeedforwardSim,
    //     Constants.ElevatorConstants.kvFeedforwardSim);



    // private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
    //         Constants.ElevatorConstants.maxVelocity,
    //         Constants.ElevatorConstants.maxAcceleration
    //     );

    // public ElevatorIOSim(){

    //     leftMotor = new DCMotorSim(
    //         LinearSystemId.createDCMotorSystem(
    //             Constants.ElevatorConstants.wristGearbox,
    //             0.025,
    //             Constants.ElevatorConstants.wristMotorReduction),
    //         Constants.ElevatorConstants.wristGearbox);


    //     leftController = new NTProfiledPIDF(
    //         "Wrist",
    //         Constants.ElevatorConstants.kpWristSim,
    //         Constants.ElevatorConstants.kiWristSim,
    //         Constants.ElevatorConstants.kdWristSim,
    //         Constants.ElevatorConstants.ksFeedforwardSim,
    //         Constants.ElevatorConstants.kvFeedforwardSim,
    //         constraints);

    //     rightMotor = new DCMotorSim(
    //         LinearSystemId.createDCMotorSystem(
    //             Constants.ElevatorConstants.wristGearbox,
    //             0.025,
    //             Constants.ElevatorConstants.wristMotorReduction),
    //         Constants.ElevatorConstants.wristGearbox);

    // }



    // @Override
    // public void ioPeriodic() {

    //     double voltage = leftController.calculate(leftMotor.getAngularPositionRad());

    //     double feedforwardVoltMaybe = feedforward.calculate(leftController.getGoal().velocity);

    //     leftMotor.setInputVoltage(voltage + feedforwardVoltMaybe);
    //     rightMotor.setInputVoltage(voltage + feedforwardVoltMaybe);

    //     leftMotor.update(Constants.WristConstants.kDt);
    //     rightMotor.update(Constants.WristConstants.kDt);

    // }



    // @Override
    // public void setTargetAngle(double _positionRad) {
    //     leftController.setGoal(new State(_positionRad, 0));
    // }



    // @Override
    // public boolean hasReachedGoal() {
    //     return leftController.atGoal();
    // }



    // @Override
    // public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    //     inputs.elevatorPosition = leftMotor.getAngularPositionRad();
    //     inputs.elevatorSetpointPosition = leftController.getSetpoint().position;
    // }
}
