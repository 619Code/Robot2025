package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    
    @AutoLog
    public static class ClimbIOInputs{
        public double ClimbPosition = 0.0;
        public double ClimbSetpointPosition = 0.0;
    }

    public void updateInputs(ClimbIOInputs inputs);

    public void ioPeriodic(double voltage);
}
