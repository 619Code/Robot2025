package frc.robot.subsystems.Intake;

public interface IntakeIO {

    public double getPosition();
    public void stopMotor();

    public void setVoltage(double voltage);
    public void update();

}
