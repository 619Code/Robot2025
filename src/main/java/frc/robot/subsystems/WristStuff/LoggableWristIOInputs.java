package frc.robot.subsystems.WristStuff;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggableWristIOInputs extends WristIO.WristIOInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("WristPosition", wristPosition);
    }

    @Override
    public void fromLog(LogTable table) {
        wristPosition = table.get("WristPosition", wristPosition);
    }

    @Override
    public LoggableWristIOInputs clone(){
        LoggableWristIOInputs copy = new LoggableWristIOInputs();
        copy.wristPosition = this.wristPosition;
        return copy;
    }
}