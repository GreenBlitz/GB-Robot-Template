package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.GBSubsystem;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.TalonFXWrapper;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;
import java.util.logging.LogRecord;

public class talonSubsystem extends GBSubsystem {

    private final TalonFXWrapper motor;

    public talonSubsystem(String logPath, int id) {
        super(logPath);
        motor = new TalonFXWrapper(id);
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        Supplier<StatusCode> statusCodeSupplier = () -> motor.applyConfiguration(talonFXConfiguration);
        Logger.recordOutput("idk", PhoenixProUtils.checkWithRetry(statusCodeSupplier, 5));
    }


    @Override
    protected void subsystemPeriodic() {

    }
}
