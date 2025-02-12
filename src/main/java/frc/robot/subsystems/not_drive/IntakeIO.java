package frc.robot.subsystems.not_drive;

public interface IntakeIO {
    public double getPosition();
    public void stopMotor();
    public void setVoltage(double voltage);
    public void update();
}
