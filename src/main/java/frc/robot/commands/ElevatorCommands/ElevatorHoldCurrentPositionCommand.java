package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorHoldCurrentPositionCommand extends Command {

  private final Elevator elevatorSubsystem;

  public ElevatorHoldCurrentPositionCommand(Elevator _sub) {
    elevatorSubsystem = _sub;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
