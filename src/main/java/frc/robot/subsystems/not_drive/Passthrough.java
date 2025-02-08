package frc.robot.subsystems.not_drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Passthrough extends SubsystemBase {

  private final SparkMax passthroughMotorL;
  private final SparkMax passthroughMotorR;

  private final double passthroughMotorSpeed = 0.5;

  public Passthrough(int passthroughMotorID_L, int passthroughMotorID_R) {

    passthroughMotorL = new SparkMax(passthroughMotorID_L, MotorType.kBrushless);
    passthroughMotorR = new SparkMax(passthroughMotorID_R, MotorType.kBrushless);

    SparkMaxConfig config_L = new SparkMaxConfig();
    config_L.idleMode(IdleMode.kBrake);

    SparkMaxConfig config_R = new SparkMaxConfig();
    config_R.idleMode(IdleMode.kBrake);
    config_R.follow(passthroughMotorL, false);

    passthroughMotorL.configure(config_L, null, null);
    passthroughMotorR.configure(config_R, null, null);
  }
  // Need to connect to Intake Start and Outake Sensor.

  //  The below two functions may not need to be public (can decide when we know the sensor
  // situation)
  public void RunPassthrough() {
    passthroughMotorL.setVoltage(Constants.PassthroughConstants.passthroughMotorVoltage);
  }

  public void HaltPassthrough() {
    passthroughMotorL.stopMotor();
  }
}
