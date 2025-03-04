package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    public static class ClimbIOInputs {
        public double climbPosition = 0.0;
    }

    public void updateInputs(ClimbIOInputsAutoLogged inputs);


    public double getPosition();
    public void stopMotor();
    public void setVoltage(double voltage);
}
