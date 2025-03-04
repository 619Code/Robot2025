package frc.robot.subsystems.Outtake;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO  {

    @AutoLog
    public static class OuttakeIOInputs {
        public boolean hasCoral;
    }

    public void updateInputs(OuttakeIOInputs inputs);

    //  Outtake
    public void runOuttakeVoltage(double voltage);
    public void stopOuttake();

    //  Dislodger
    public void setDislodgerVoltage(double voltage);
    //  There is no stopDislodger() right here because it is profiled. It profiles to a stop.


    //  Proximity sensor
//    public boolean hasCoral();

}
