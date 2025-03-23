package frc.robot.commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorFineTuningCommand extends Command {

  private final Elevator elevatorSubsystem;

  private final DoubleSupplier joystick;

  public ElevatorFineTuningCommand(Elevator _sub, DoubleSupplier _joystick) {
    elevatorSubsystem = _sub;
    joystick = _joystick;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.shiftTargetPosition(joystick.getAsDouble() * 0.01);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
