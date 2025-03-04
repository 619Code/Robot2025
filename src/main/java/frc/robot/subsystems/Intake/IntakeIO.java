package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs{
        public double position;
    }

  //  public void updateInputs(IntakeIOInputsAutoLogged inputs);

    public double getPosition();
    public void stopMotor();

    public void setExtensionMotorVoltage(double voltage);
    public void setIntakeMotorVoltage(double voltage);

}
