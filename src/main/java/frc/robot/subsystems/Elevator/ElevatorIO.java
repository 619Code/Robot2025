package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.ElevatorConstants.ElevatorHeight;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPositionRotations = 0.0;

        public double elevatorSetpointPositionMeters = 0.0;
        public double elevatorHeightMeters = ElevatorHeight.HOME.heightMeters;
        public double elevatorVelocityMPS = 0;

        public double elevatorGoalMeters = ElevatorHeight.HOME.heightMeters;
        public ElevatorHeight elevatorGoalEnum = ElevatorHeight.HOME;

        public double elevatorVoltage = 0.0;
    }

    public void updateInputs(ElevatorIOInputsAutoLogged inputs);

    public void runVoltage(double voltage);
}
