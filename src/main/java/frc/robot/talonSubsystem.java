package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.utils.GBSubsystem;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.function.Supplier;

public class talonSubsystem extends GBSubsystem {

    private final TalonFXWrapper motor;

    public talonSubsystem(String logPath, int id) {
        super(logPath);
        motor = new TalonFXWrapper(id);
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        Supplier<StatusCode> statusCodeSupplier = () -> motor.applyConfiguration(talonFXConfiguration);
        PhoenixProUtils.checkWithRetry(statusCodeSupplier, 5);
    }

    @Override
    protected void subsystemPeriodic() {

    }
}
