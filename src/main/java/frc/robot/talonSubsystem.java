package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.utils.GBSubsystem;
import frc.utils.devicewrappers.TalonFXWrapper;
import frc.utils.brakestate.BrakeStateManager;

public class talonSubsystem extends GBSubsystem {

    private final TalonFXWrapper motor;

    public talonSubsystem(String logPath) {
        super(logPath);
        motor = new TalonFXWrapper(11);
        BrakeStateManager.addSubsystem(this);
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake) {
            motor.setNeutralMode(NeutralModeValue.Brake);
        }
        else motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    protected void subsystemPeriodic() {

    }
}
