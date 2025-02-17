package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class NTProfiledFlex  {

    //  Hardware

    private final SparkFlex motor;
    private final SparkClosedLoopController onBoardController;
    private final AbsoluteEncoder encoder;

    //  Control

    private final TrapezoidProfile trapezoidProfile;

    private TrapezoidProfile.State currentGoal;
    private TrapezoidProfile.State currentSetpoint;

    // Network tables

    private final DoubleEntry kpEntry;
    private final DoubleEntry kiEntry;
    private final DoubleEntry kdEntry;

    private double currentKp;
    private double currentKi;
    private double currentKd;


    public NTProfiledFlex(String _uniqueName, int _motorID, double _kp, double _ki, double _kd, SparkFlexConfig _config, TrapezoidProfile.Constraints _contraints){

        //  Make config
        _config.closedLoop
            .p(_kp)
            .i(_ki)
            .d(_kd);


        motor = new SparkFlex(_motorID, MotorType.kBrushless);
        onBoardController = motor.getClosedLoopController();

        motor.configure(_config, null,null);

        trapezoidProfile = new TrapezoidProfile(_contraints);

        encoder = motor.getAbsoluteEncoder();

        currentSetpoint = new TrapezoidProfile.State(encoder.getPosition(), 0);
        currentGoal = new TrapezoidProfile.State(currentSetpoint.position, currentSetpoint.velocity);


        //  Network tables

        kpEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_kp").getEntry(_kp);
        kiEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_ki").getEntry(_ki);
        kdEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_kd").getEntry(_kd);

        kpEntry.set(_kp);
        kiEntry.set(_ki);
        kdEntry.set(_kd);

        currentKp = _kp;
        currentKi = _ki;
        currentKd = _kd;

    }


    private void handleNetworkTables(){

        boolean valueHasBeenUpdated = false;

        double newKp = kpEntry.get();
        double newKi = kiEntry.get();
        double newKd = kdEntry.get();

        valueHasBeenUpdated |= currentKp != newKp;
        valueHasBeenUpdated |= currentKi != newKi;
        valueHasBeenUpdated |= currentKd != newKd;

        currentKp = newKp;
        currentKi = newKi;
        currentKd = newKd;

        if(valueHasBeenUpdated){
            SparkFlexConfig newConfig = new SparkFlexConfig();
            newConfig.closedLoop
                .p(currentKp)
                .i(currentKi)
                .d(currentKd);

            motor.configure(newConfig, null, null);

            System.out.println("Updating pid values of NTProfiledFlex to, p: " + currentKp + ",  i: " + currentKi + ",  d: " + currentKd);

        }
    }


    //  NOTICE: Come back here and uncomment stuff so that thangs can actually move
    public void update(){

        handleNetworkTables();

        currentSetpoint = trapezoidProfile.calculate(Constants.WristConstants.kDt, currentSetpoint, currentGoal);

        double feedforward = 0.3 * Math.cos(encoder.getPosition() + Math.PI);

        // onBoardController.setReference(
        //     currentSetpoint.position,
        //     ControlType.kPosition,
        //     ClosedLoopSlot.kSlot0
        //     Constants.WristConstants.ksFeedforward * Math.signum(currentSetpoint.velocity) +
        //     Constants.WristConstants.kvFeedforward * currentSetpoint.velocity
        // );



        onBoardController.setReference(
            currentSetpoint.velocity,
            ControlType.kVoltage,
            ClosedLoopSlot.kSlot0,
            feedforward
        );

        // onBoardController.setReference(
        //     currentSetpoint.position,
        //     ControlType.kPosition,
        //     ClosedLoopSlot.kSlot0,
        //     0.3 * Math.cos(encoder.getPosition() + Math.PI)
        // );

    }

    public void goToState(State _state){
        currentGoal = _state;
    }

    public boolean hasReachedGoal(){
        return trapezoidProfile.isFinished(0);
    }

    public AbsoluteEncoder getAbsoluteEncoder(){
        return motor.getAbsoluteEncoder();
    }

    public SparkFlex getMotor(){
        return motor;
    }
}
