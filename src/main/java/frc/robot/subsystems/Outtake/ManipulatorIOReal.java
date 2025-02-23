// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Outtake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ManipulatorIOReal implements ManipulatorIO {


  public final SparkMax outMax;
  public final SparkMax dislodgeMax;

  public final DigitalInput intakeProximitySensor;

  public ManipulatorIOReal() {

    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(10);

    outMax = new SparkMax(Constants.OuttakeConstants.outtakeMotorId, MotorType.kBrushless);


    dislodgeMax = new SparkMax(Constants.OuttakeConstants.dislodgerMotorId, MotorType.kBrushless);

    intakeProximitySensor = new DigitalInput(Constants.OuttakeConstants.kIntakeSensorPort);
  }


  @Override
  public void runOuttakeOut() {
    outMax.setVoltage(Constants.OuttakeConstants.outtakeVoltage);
  }
  @Override
  public void runOuttakeIn() {
    outMax.setVoltage(Constants.OuttakeConstants.intakeVoltage);
  }


  @Override
  public void runDislodger() {
    dislodgeMax.setVoltage(Constants.OuttakeConstants.dislodgerVoltage);
  }


  @Override
  public void stopOuttake() {
    outMax.stopMotor();
  }
  @Override
  public void stopDislodger(){
    dislodgeMax.stopMotor();
  }


  private boolean hasCoral() {
    return !intakeProximitySensor.get();
  }


  @Override
  public void updateInputs(OuttakeIOInputs inputs){
    inputs.hasCoral = hasCoral();
  }


  @Override
  public void ioPeriodic() {
      //  Nothing
  }
}
