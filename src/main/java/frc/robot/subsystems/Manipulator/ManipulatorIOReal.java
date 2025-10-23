// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ManipulatorIOReal implements ManipulatorIO {


  public final SparkMax outMax;

  private final DigitalInput intakeProximitySensor;

  public ManipulatorIOReal(int _outtakeMotorID) {

    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(10);

    outMax = new SparkMax(_outtakeMotorID, MotorType.kBrushless);

    intakeProximitySensor = new DigitalInput(Constants.ManipulatorConstants.kIntakeSensorPort);
  }


  @Override
  public void runOuttakeVoltage(double voltage) {
    outMax.setVoltage(voltage);
  }


  @Override
  public void stopOuttake() {
    outMax.stopMotor();
  }


  private boolean hasCoral() {
    return !intakeProximitySensor.get();
  }


  @Override
  public void updateInputs(OuttakeIOInputs inputs){
    inputs.hasCoral = hasCoral();
  }


  @Override
  public void setDislodgerVoltage(double _voltage) {
    // no op
  }
}
