package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTProfiledPIDF {

    private final ProfiledPIDController controller;





    private final DoubleEntry kpEntry;
    private final DoubleEntry kiEntry;
    private final DoubleEntry kdEntry;
    private final DoubleEntry ksEntry;
    private final DoubleEntry kvEntry;

    public NTProfiledPIDF(String _uniqueName, double _kp, double _ki, double _kd, double _ks, double _kv, Constraints _constraints){

        controller = new ProfiledPIDController(_kp, _ki, _kd, _constraints);

        controller.setIZone(0.1);

        kpEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_kp").getEntry(_kp);
        kiEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_ki").getEntry(_ki);
        kdEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_kd").getEntry(_kd);

        ksEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_ks").getEntry(_ks);
        kvEntry = NetworkTableInstance.getDefault().getDoubleTopic(_uniqueName + "_kv").getEntry(_kv);

        kpEntry.set(_kp);
        kiEntry.set(_ki);
        kdEntry.set(_kd);

        ksEntry.set(_ks);
        kvEntry.set(_kv);

    }

    public double calculate(double measurement){

        controller.setP(kpEntry.get());
        controller.setI(kiEntry.get());
        controller.setD(kdEntry.get());

        //  This controller.calculate is purposefully getting called before the controller.getSetpoint a few lines below.
        //  This is because the setpoint probably gets updated in controller.calculate
        double output = controller.calculate(measurement);

        State state = controller.getSetpoint();

        double feedforward = ksEntry.get() * Math.signum(state.velocity) + kvEntry.get() * state.velocity;

        return output + feedforward;
    }

    public void setGoal(State value){
        controller.setGoal(value);
    }

    public void setTolerance(double _val){
        controller.setTolerance(_val);
    }

    public State getGoal(){
        return controller.getGoal();
    }

    public State getSetpoint(){
        return controller.getSetpoint();
    }

    public boolean atGoal(){
        return controller.atGoal();
    }
}
