package frc.robot.subsystems.WristStuff;



import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristDirectControlSubsystem extends SubsystemBase {

    private final SparkFlex wristFlex;

    private final AbsoluteEncoder wristEncoder;

    public WristDirectControlSubsystem(int wristMotorID) {

        //  This should be able to be default. PID values get set in the constructor fo NTProfiledFlex
        SparkFlexConfig config = new SparkFlexConfig();
        config.absoluteEncoder.positionConversionFactor(2.0 * Math.PI);
        config.absoluteEncoder.zeroOffset(Constants.WristConstants.zeroOffset);
        config.idleMode(IdleMode.kCoast);


        SoftLimitConfig softLimits = new SoftLimitConfig();
        softLimits.forwardSoftLimit(Constants.WristConstants.softUpperLimitRotations);
        softLimits.reverseSoftLimit(Constants.WristConstants.softLowerLimitRotations);
        softLimits.forwardSoftLimitEnabled(true);
        softLimits.reverseSoftLimitEnabled(true);
        config.softLimit.apply(softLimits);



        wristFlex = new SparkFlex(wristMotorID, MotorType.kBrushless);
     //   onBoardController = wristFlex.getClosedLoopController();

        wristFlex.configure(config, null,PersistMode.kPersistParameters);
        wristEncoder = wristFlex.getAbsoluteEncoder();

    }


    @Override
    public void periodic() {

      System.out.println("Wrist encoder position radians: " + wristEncoder.getPosition());

    }

    public void runPercentage(double percentSpeed){
        // onBoardController.setReference(
        //     velocity,
        //     ControlType.kVelocity,
        //     ClosedLoopSlot.kSlot0);
        wristFlex.set(percentSpeed);
    }
}
