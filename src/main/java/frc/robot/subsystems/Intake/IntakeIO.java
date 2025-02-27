package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double intakePosition = 0.0;
        public double intakeSetpointPosition = 0.0;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void ioPeriodic();

    public void setTargetPosition(double _position);

    public boolean hasReachedGoal();

}
