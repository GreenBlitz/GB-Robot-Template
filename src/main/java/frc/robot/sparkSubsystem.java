package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.utils.GBSubsystem;
import frc.utils.brakestate.BrakeStateManager;

public class sparkSubsystem extends GBSubsystem {
    private final CANSparkMax motor;

    public sparkSubsystem(String logPath) {
        super(logPath);
        motor = new CANSparkMax(5, MotorType.kBrushless);
        BrakeStateManager.addSubsystem(this);
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake){
            motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        }
        else motor.setIdleMode(CANSparkBase.IdleMode.kCoast);

    }

    @Override
    protected void subsystemPeriodic() {

    }
}
