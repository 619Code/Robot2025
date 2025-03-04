// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Outtake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ManipulatorIOReal implements ManipulatorIO {


  public final SparkMax outMax;
  public final SparkMax dislodgeMax;

  private final DigitalInput intakeProximitySensor;

  private final TrapezoidProfile dislodgerTrapezoidProfile;

  private State currentDislodgerSetpoint;
  private State currentDislodgerGoal;

  private final DoublePublisher currentDislodgerVoltage;

  public ManipulatorIOReal() {

    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(10);

    outMax = new SparkMax(Constants.OuttakeConstants.outtakeMotorId, MotorType.kBrushless);


    dislodgeMax = new SparkMax(Constants.OuttakeConstants.dislodgerMotorId, MotorType.kBrushless);


    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(15, 30);
    dislodgerTrapezoidProfile = new TrapezoidProfile(constraints);

    currentDislodgerSetpoint = new State(0, 0);
    currentDislodgerGoal = new State(0, 0);



    currentDislodgerVoltage = NetworkTableInstance.getDefault().getDoubleTopic("Dislodger/Current voltage").publish();
    currentDislodgerVoltage.set(0);


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
  public void runDislodger(boolean invert) {
    currentDislodgerGoal = new State(
      Constants.OuttakeConstants.dislodgerVoltage * (invert ? -1 : 1),
       0
    );
  }


  @Override
  public void stopOuttake() {
    outMax.stopMotor();
  }
  @Override
  public void stopDislodger(){
    currentDislodgerGoal = new State(0, 0);
  }


  private boolean hasCoral() {
    return !intakeProximitySensor.get();
  }


  @Override
  public void updateInputs(OuttakeIOInputs inputs){
    inputs.hasCoral = hasCoral();
  }


  @Override
  public void periodic() {

    currentDislodgerSetpoint = dislodgerTrapezoidProfile.calculate(Constants.WristConstants.kDt, currentDislodgerSetpoint, currentDislodgerGoal);

    double voltage = currentDislodgerSetpoint.position;

    currentDislodgerVoltage.set(voltage);

    dislodgeMax.setVoltage(voltage);

  }
}
