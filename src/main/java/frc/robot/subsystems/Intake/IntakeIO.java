package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs{
        public double intakeExtensionPosition;
        public double intakeMotorSpeedRadSec;
    }

    public void updateInputs(IntakeIOInputsAutoLogged inputs);

    public void stopExtensionMotor();

    public void setExtensionMotorVoltage(double voltage);
    public void setIntakeMotorVoltage(double voltage);

}
