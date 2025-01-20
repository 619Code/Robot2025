package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class LoggedCommand extends Command {

    private final Command command;
    private final String commandLogKey;
    private final String commandName;
    private static boolean isLoggingEnabled = false;

    public LoggedCommand(Command command) {
        this.addRequirements(command.getRequirements());

        this.command = command;
        this.commandLogKey = "Commands/" + command.getName();
        this.commandName = command.getName();
    }

    public void setLoggingEnabled(boolean enabled) {
        LoggedCommand.isLoggingEnabled = enabled;
    }


    private void logMessage(String message) {
        Logger.recordOutput(commandLogKey, String.format("[%s] %s", commandName, message));
    }


    @Override
    public void initialize() {
        if (LoggedCommand.isLoggingEnabled)
        {
            logMessage("Starting initialization.");
            this.command.initialize();
            logMessage("Initialization finished.");
        } else {
            this.command.initialize();
        }
    }

    @Override
    public void execute() {
        if (LoggedCommand.isLoggingEnabled)
        {
            logMessage("Executing.");
        }
        this.command.execute();

    }

    @Override
    public void end(boolean interrupted) {
        if (LoggedCommand.isLoggingEnabled)
        {
            logMessage("Starting end. Interrupted = ." + interrupted);
            this.command.end(interrupted);
            logMessage("End finished.");
        } else {
            this.command.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = command.isFinished();

        if (LoggedCommand.isLoggingEnabled)
        {
            logMessage("Has finished");
        }

        return isFinished;
    }

}
