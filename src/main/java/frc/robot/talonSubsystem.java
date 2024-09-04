package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import frc.utils.GBSubsystem;
import frc.utils.MotorSubsystem;
import frc.utils.devicewrappers.TalonFXWrapper;
import frc.utils.brakestate.BrakeStateManager;

public class talonSubsystem extends MotorSubsystem {

    private final TalonFXWrapper motor;

    public talonSubsystem(String logPath, int id) {
        super(logPath);
        motor = new TalonFXWrapper(id);

    }

    @Override
    public void setBrake(boolean brake) {
        NeutralModeValue x = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.setNeutralMode(x);
    }

    @Override
    protected void subsystemPeriodic() {

    }
}