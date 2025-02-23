package frc.robot.subsystems.Outtake;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO  {

    @AutoLog
    public static class OuttakeIOInputs {
        public boolean hasCoral;
    }

    public void updateInputs(OuttakeIOInputs inputs);

    public void ioPeriodic();

    //  Outtake
    public void runOuttakeOut();
    public void runOuttakeIn();
    public void stopDislodger();

    //  Dislodger
    public void runDislodger(boolean invert);
    public void stopOuttake();

    //  Proximity sensor
//    public boolean hasCoral();

}
