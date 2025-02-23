// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake.Manipulator;

public class DislodgeAlgaeCommand extends Command {

  public Manipulator manipulator;

  private final boolean invert;

  public DislodgeAlgaeCommand(Manipulator _manipulator, boolean _invert) {

    manipulator = _manipulator;

    invert = _invert;

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.runDislodger(invert);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stopDislodger();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
