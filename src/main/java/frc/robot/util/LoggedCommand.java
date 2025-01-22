package frc.robot.util;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LoggedCommand extends Command {

    private final Command command;
    private final String commandLogKey;
    private final String commandName;
    private double lastTimestamp = 0;
    private String lastEntry = null;

    private static boolean isLoggingEnabled = false;
    private static boolean cacheValues = false;
    private static String allCommmandsLogKey = "LoggedCommands/all";


    public LoggedCommand(Command command) {
        this(command.getName(), command);
    }

    public LoggedCommand(String commandName, Command command) {
        this.addRequirements(command.getRequirements());

        this.command = command;
        this.commandLogKey = "LoggedCommands/" + commandName;
        this.commandName = commandName;
    }

    public static void configureCommandLogging(boolean enabled, boolean cacheValues) {
        LoggedCommand.isLoggingEnabled = enabled;
        LoggedCommand.cacheValues = cacheValues;
    }


    private void logMessage(String message) {
        String toLog;
        if (LoggedCommand.cacheValues) {
            // If the new value matches the previous, AK won't update the value.
            // It acts like a latch.
            toLog = String.format("[%s] %s", commandName, message);
        } else {
            toLog = String.format("%f [%s] %s", Timer.getTimestamp(), commandName, message);
        }

        double timestamp = Logger.getTimestamp();
        if (Math.abs(timestamp - lastTimestamp) <= 0.02 && lastEntry != null) {
            // We've already logged something in this time slice.
            // We need to append to the string instead of replacing it.
            toLog = String.format("%s\n%s", lastEntry, toLog);
        }

        lastTimestamp = timestamp;
        lastEntry = toLog;

        Logger.recordOutput(LoggedCommand.allCommmandsLogKey, toLog);
        Logger.recordOutput(commandLogKey, toLog);
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
            logMessage("Starting end. Interrupted = " + interrupted);
            this.command.end(interrupted);
            logMessage("End finished.");
        } else {
            this.command.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = command.isFinished();

        if (LoggedCommand.isLoggingEnabled && isFinished)
        {
            logMessage("Has finished");
        }

        return isFinished;
    }

}
