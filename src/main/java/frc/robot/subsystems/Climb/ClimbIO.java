package frc.robot.subsystems.Climb;

public interface ClimbIO {
    public double getPosition();
    public void stopMotor();
    public void setVoltage(double voltage);
    public void update();
}
