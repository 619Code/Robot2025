package frc.robot.subsystems.Passthrough;

import org.littletonrobotics.junction.AutoLog;

public interface PassthroughIO {

    @AutoLog
    public static class PassthroughIOInputs {
        //  This is useless, I'm just setting this stuff up now so I don't have to later.
        //  We do not need the voltage in here.
        public double motorVoltage;
    }

    public void updateInputs(PassthroughIOInputsAutoLogged inputs);

    public void setVoltage(double voltage);
    public void stopMotors();

}
