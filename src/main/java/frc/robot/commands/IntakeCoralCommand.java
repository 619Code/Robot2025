// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake.Manipulator;
import frc.robot.subsystems.Passthrough.Passthrough;

public class IntakeCoralCommand extends Command {

  public Manipulator manipulator;
  public Passthrough passthrough;

  public IntakeCoralCommand(Manipulator _manipulator, Passthrough _passthrough) {

    manipulator = _manipulator;
    passthrough = _passthrough;

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.runOuttakeIn();
    passthrough.RunPassthrough();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    manipulator.stopOuttake();
    passthrough.HaltPassthrough();
  }

  @Override
  public boolean isFinished() {
    return manipulator.isDetectingCoral();
  }
}
