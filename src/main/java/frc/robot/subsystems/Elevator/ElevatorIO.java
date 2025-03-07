package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPositionRotations = 0.0;

        public double elevatorSetpointPositionMeters = 0.0;
        public double elevatorHeightMeters = 0.0;

        public double elevatorGoalMeters = 0.0;

        public double elevatorVoltage = 0.0;
    }

    public void updateInputs(ElevatorIOInputsAutoLogged inputs);

    public void runVoltage(double voltage);
}
