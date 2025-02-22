package frc.robot.subsystems.WristStuff;

import static frc.robot.util.SparkUtil.ifOk;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.NTProfiledPIDF;

public class WristIOReal implements WristIO {

    private final SparkFlex wristFlex;
    private final NTProfiledPIDF controller;
 //   private final ArmFeedforward deviousClown;

    private final double maxVoltage = 5.0;

    private final TrapezoidProfile.State passthroughState = new TrapezoidProfile.State(Constants.WristConstants.passthroughPositionRad, 0);
    private final TrapezoidProfile.State L1State = new TrapezoidProfile.State(Constants.WristConstants.L1PositionRad, 0);
    private final TrapezoidProfile.State L2L3State = new TrapezoidProfile.State(Constants.WristConstants.L2L3PositionRad, 0);
    private final TrapezoidProfile.State L4State = new TrapezoidProfile.State(Constants.WristConstants.L4PositionRad, 0);

    private final AbsoluteEncoder wristEncoder;


    private final DoublePublisher currentGoalPosPub;
    private final DoublePublisher currentVelocity;
    private final DoublePublisher desiredVelocity;
    private final DoublePublisher desiredPosition;
    private final DoublePublisher currentVoltage;


    private final DoublePublisher feedForwardProportion;
    private final DoublePublisher positionalError;

    private final BooleanPublisher voltageClamped;


    public WristIOReal() {

        //  This should be able to be default. PID values get set in the constructor fo NTProfiledFlex
        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        config.absoluteEncoder.positionConversionFactor(2.0 * Math.PI);
        config.absoluteEncoder.zeroOffset(Constants.WristConstants.zeroOffset);

        config.smartCurrentLimit(60);

        // config.periodicFramePeriod(PeriodicFrame.kStatus5, 20);
        // config.periodicFramePeriod(PeriodicFrame.kStatus6, 20);


  //      config.externalEncoder.measurementPeriod(1);

        config.idleMode(IdleMode.kCoast);


        SoftLimitConfig softLimits = new SoftLimitConfig();
        softLimits.forwardSoftLimit(Constants.WristConstants.softUpperLimitRotations);
        softLimits.reverseSoftLimit(Constants.WristConstants.softLowerLimitRotations);
        softLimits.forwardSoftLimitEnabled(true);
        softLimits.reverseSoftLimitEnabled(true);
        config.softLimit.apply(softLimits);

        //  Set constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                Constants.WristConstants.maxVelocity,
                Constants.WristConstants.maxAcceleration);

        //  Create motor
        wristFlex = new SparkFlex(
            Constants.WristConstants.wristMotorID,
            MotorType.kBrushless
        );
        wristFlex.configure(config, null, null);



        controller = new NTProfiledPIDF(
            "Wrist",
            Constants.WristConstants.kpWrist,
            Constants.WristConstants.kiWristSim,
            Constants.WristConstants.kiWrist,
            Constants.WristConstants.ksFeedforward,
            Constants.WristConstants.kvFeedforward,
            constraints);

       //     deviousClown = new ArmFeedforward(Constants.WristConstants.ksFeedforward, );

        wristEncoder = wristFlex.getAbsoluteEncoder();

        controller.setGoal(new State(wristEncoder.getPosition(), 0));


        currentGoalPosPub = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex goal pos").publish();
        currentVelocity = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex current velocity").publish();
        desiredVelocity = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex desired velocity").publish();
        desiredPosition = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex desired position").publish();
        currentVoltage = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex current voltage").publish();
        feedForwardProportion = NetworkTableInstance.getDefault().getDoubleTopic("Wrist feedforward proportion").publish();
        positionalError = NetworkTableInstance.getDefault().getDoubleTopic("Wrist position error").publish();
        voltageClamped = NetworkTableInstance.getDefault().getBooleanTopic("Wrist voltage clamped").publish();

        currentGoalPosPub.set(passthroughState.position);
        currentVelocity.set(0);
        desiredVelocity.set(0);
        desiredPosition.set(0);
        currentVoltage.set(0);
        feedForwardProportion.set(0);
        positionalError.set(0);
        voltageClamped.set(false);

    }


    @Override
    public void ioPeriodic() {

        double voltage = controller.calculate(wristEncoder.getPosition());
        double feedforward = 0.3 * Math.cos(wristEncoder.getPosition() + Math.PI);

        //  Logging
        double absValSum = Math.abs(feedforward) + Math.abs(voltage);
        feedForwardProportion.set(Math.abs(feedforward) / Math.abs(absValSum));
        //

        voltage += feedforward;

        voltageClamped.set(Math.abs(voltage) >= maxVoltage);

        voltage = Math.min(voltage, maxVoltage);
        voltage = Math.max(voltage, -maxVoltage);

        wristFlex.setVoltage(voltage);


        currentVelocity.set(wristEncoder.getVelocity());
        desiredVelocity.set(controller.getSetpoint().velocity);
        desiredPosition.set(controller.getSetpoint().position);
        positionalError.set(controller.getSetpoint().position - wristEncoder.getPosition());
        currentVoltage.set(voltage);

      //  The encoder value should natively be in radians now
  //    System.out.println("Wrist encoder position radians: " + wristEncoder.getPosition());

    }

    @Override
    public void goToPassthroughAngle(){
        controller.setGoal(passthroughState);
        currentGoalPosPub.set(passthroughState.position);
    }

    @Override
    public void goToL1Angle() {
        controller.setGoal(L1State);
        currentGoalPosPub.set(L1State.position);
    }

    @Override
    public void goToL2L3Angle(){
        controller.setGoal(L2L3State);
        currentGoalPosPub.set(L2L3State.position);
    }

    @Override
    public void goToL4Angle(){
        controller.setGoal(L4State);
        currentGoalPosPub.set(L4State.position);
    }

    @Override
    public boolean hasReachedGoal(){
         return controller.atGoal();
    }


    @Override
    public void updateInputs(WristIOInputs inputs){
        ifOk(wristFlex, wristEncoder::getPosition, (value) -> inputs.wristPosition = value);
    }
}




// package frc.robot.subsystems.WristStuff;

// import static frc.robot.util.SparkUtil.ifOk;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import com.revrobotics.spark.config.SoftLimitConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.util.NTProfiledFlex;

// public class WristIOReal extends SubsystemBase implements WristIO {

//     private final NTProfiledFlex wristFlex;


//     private final TrapezoidProfile.State passthroughState = new TrapezoidProfile.State(Constants.WristConstants.passthroughPositionRad, 0);
//     private final TrapezoidProfile.State L1State = new TrapezoidProfile.State(Constants.WristConstants.L1PositionRad, 0);
//     private final TrapezoidProfile.State L2L3State = new TrapezoidProfile.State(Constants.WristConstants.L2L3PositionRad, 0);
//     private final TrapezoidProfile.State L4State = new TrapezoidProfile.State(Constants.WristConstants.L4PositionRad, 0);

//     private final AbsoluteEncoder wristEncoder;



//     DoublePublisher currentGoalPosPub;


//     public WristIOReal(int wristMotorID) {

//         //  This should be able to be default. PID values get set in the constructor fo NTProfiledFlex
//         SparkFlexConfig config = new SparkFlexConfig();
//         config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

//         config.absoluteEncoder.positionConversionFactor(2.0 * Math.PI);
//         config.absoluteEncoder.zeroOffset(Constants.WristConstants.zeroOffset);
//         config.idleMode(IdleMode.kCoast);


//         SoftLimitConfig softLimits = new SoftLimitConfig();
//         softLimits.forwardSoftLimit(Constants.WristConstants.softUpperLimitRotations);
//         softLimits.reverseSoftLimit(Constants.WristConstants.softLowerLimitRotations);
//         softLimits.forwardSoftLimitEnabled(true);
//         softLimits.reverseSoftLimitEnabled(true);
//         config.softLimit.apply(softLimits);

//         //  Set constraints
//         TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
//                 Constants.WristConstants.maxVelocity,
//                 Constants.WristConstants.maxAcceleration);

//         //  Create motor
//         wristFlex = new NTProfiledFlex(
//             "WristFlex",
//             wristMotorID,
//             Constants.WristConstants.kpWrist,
//             Constants.WristConstants.kiWrist,
//             Constants.WristConstants.kdWrist,
//             config,
//             constraints
//         );

//         wristEncoder = wristFlex.getAbsoluteEncoder();


//         currentGoalPosPub = NetworkTableInstance.getDefault().getDoubleTopic("WristFlex goal pos").publish();
//         currentGoalPosPub.set(passthroughState.position);

//     }


//     @Override
//     public void periodic() {

//       wristFlex.update();

//       //  The encoder value should natively be in radians now
//   //    System.out.println("Wrist encoder position radians: " + wristEncoder.getPosition());

//     }

//     @Override
//     public void goToPassthroughAngle(){
//         wristFlex.goToState(passthroughState);
//         currentGoalPosPub.set(passthroughState.position);
//     }

//     @Override
//     public void goToL1Angle() {
//         wristFlex.goToState(L1State);
//         currentGoalPosPub.set(L1State.position);
//     }

//     @Override
//     public void goToL2L3Angle(){
//         wristFlex.goToState(L2L3State);
//         currentGoalPosPub.set(L2L3State.position);
//     }

//     @Override
//     public void goToL4Angle(){
//         wristFlex.goToState(L4State);
//         currentGoalPosPub.set(L4State.position);
//     }

//     @Override
//     public boolean hasReachedGoal(){
//          return wristFlex.hasReachedGoal();
//     }


//     @Override
//     public void updateInputs(WristIOInputs inputs){
//         ifOk(wristFlex.getMotor(), wristEncoder::getPosition, (value) -> inputs.wristPosition = value);
//     }
// }
