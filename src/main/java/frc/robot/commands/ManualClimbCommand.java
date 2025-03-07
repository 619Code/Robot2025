package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb.Climb;

public class ManualClimbCommand extends Command{

  private final Climb climbSub;
  DoubleSupplier rateSupplier;

  public ManualClimbCommand(Climb climbSub, DoubleSupplier rateSupplier) {
    this.climbSub = climbSub;
    this.rateSupplier = rateSupplier;
    addRequirements(climbSub);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    var rate = rateSupplier.getAsDouble();

    rate = Math.abs(rate) <= .1 ? 0 : rate;

    DoubleSupplier voltageSupplier = () -> rateSupplier.getAsDouble() * Constants.ClimbConstants.maxVoltage;
    climbSub.setTargetVoltage(voltageSupplier.getAsDouble());

  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
      return InterruptionBehavior.kCancelSelf; // or InterruptionBehavior.kCancelIncoming
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
