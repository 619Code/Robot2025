package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPosition = 0.0;
        public double elevatorSetpointPosition = 0.0;
    }

    public void updateInputs(ElevatorIOInputsAutoLogged inputs);

    public void ioPeriodic();


    public void setTargetAngle(double _positionRad);

    public boolean hasReachedGoal();
}
