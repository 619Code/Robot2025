// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ManipulatorIOSim implements ManipulatorIO {


  public final DCMotorSim outMotor;
//  public final DCMotorSim dislodgeMotor;

  public ManipulatorIOSim() {

    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(10);

    outMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Constants.ManipulatorConstants.outtakeMotorGearbox,
                0.025,
                Constants.ManipulatorConstants.outtakeMotorReduction),
            Constants.ManipulatorConstants.outtakeMotorGearbox);


    // dislodgeMotor = new DCMotorSim(
    //   LinearSystemId.createDCMotorSystem(
    //       Constants.ManipulatorConstants.dislodgeMotorGearbox,
    //       0.025,
    //       Constants.ManipulatorConstants.dislodgeMotorReduction),
    //   Constants.ManipulatorConstants.dislodgeMotorGearbox);

  }


  @Override
  public void runOuttakeVoltage(double voltage) {
    outMotor.setInputVoltage(voltage);
  }


  @Override
  public void stopOuttake() {
    outMotor.setInputVoltage(0.0);
  }


  private boolean hasCoral() {
    return false;
  }


  @Override
  public void updateInputs(OuttakeIOInputs inputs){
    inputs.hasCoral = hasCoral();
  }


  @Override
  public void setDislodgerVoltage(double _voltage) {

    //dislodgeMotor.setInputVoltage(_voltage);

  }
}
