// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator.Manipulator;

public class ManipulatorIntakeCoralCommand extends Command {

  public Manipulator manipulator;
  public ManipulatorIntakeCoralCommand(Manipulator _manipulator) {

    manipulator = _manipulator;

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.runOuttakeIn();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stopOuttake();
  }

  @Override
  public boolean isFinished() {
    return manipulator.isDetectingCoral();
  }
}
