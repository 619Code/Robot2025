package frc.robot.subsystems.not_drive.motors;

public interface IntakeArmIO {
    public double getPosition();
    public void stopMotor();
    public void setVoltage(double voltage);
    public void update();
}
