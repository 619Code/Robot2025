package frc.robot.subsystems.not_drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final SparkMax elevatorMotor;
  private final CANcoder elevatorEncoder;

  public Elevator(int elevatorMotorID, int encoderID) {

    elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);

    elevatorEncoder = new CANcoder(encoderID);
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.MagnetOffset =
        Constants.ElevatorConstants.elevatorAbsoluteEncoderOffset;
    canCoderConfiguration.MagnetSensor.SensorDirection =
        Constants.ElevatorConstants.elevatorEncoderPositiveDirection;
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    elevatorEncoder.getConfigurator().apply(canCoderConfiguration);
  }

  @Override
  public void periodic() {}
}
