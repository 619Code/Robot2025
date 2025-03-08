package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.Constants.WristConstants.WristAngleRad;
import frc.robot.commands.ManipulatorIntakeCoralCommand;
import frc.robot.commands.ElevatorCommands.ElevatorGoToPositionPositionCommand;
import frc.robot.commands.WristCommands.WristGoToPositionCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.WristStuff.Wrist;

public class FunnelIntakeCommands {

    public static Command FunnelIntakeCommandCreator(Elevator elevator, Wrist wrist, Manipulator manipulator){

        Command command = new ConditionalCommand(
            Commands.sequence(
                whichFunnelIntakeCommand(elevator, wrist, manipulator,true)
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            Commands.sequence(
                whichFunnelIntakeCommand(elevator, wrist, manipulator, false)
            ),
            () -> {return elevator.getCurrentGoal() == ElevatorHeight.HOME;}
        );

        return command;

    }

    private static Command whichFunnelIntakeCommand(Elevator elevator, Wrist wrist, Manipulator manipulator, boolean isAlreadyAtHome){

        WristAngleRad firstWristAngle;

        if(isAlreadyAtHome){
            firstWristAngle = WristAngleRad.FREEHANG;
        }else{
            firstWristAngle = WristAngleRad.L2L3;
        }

        return Commands.sequence(
            new WristGoToPositionCommand(wrist, firstWristAngle),
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.FUNNEL),
            new WristGoToPositionCommand(wrist, WristAngleRad.FUNNEL_ANGLE),
            new ManipulatorIntakeCoralCommand(manipulator),
            new WristGoToPositionCommand(wrist, WristAngleRad.L2L3),
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.HOME)
        );
    }

}
