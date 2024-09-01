package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.GBSubsystem;
import frc.utils.devicewrappers.SparkMaxConfiguration;
import frc.utils.devicewrappers.SparkMaxWrapper;

public class sparkmaxSubsystem extends GBSubsystem {
    private final SparkMaxWrapper motor;

    public sparkmaxSubsystem(String logPath) {
        super(logPath);
        motor = new SparkMaxWrapper(4, CANSparkLowLevel.MotorType.kBrushless);
        SparkMaxConfiguration config = new SparkMaxConfiguration();
        config.slot0.withP(1);
    }

    public void moveto(Rotation2d target){

    }



    @Override
    protected void subsystemPeriodic() {

    }
}
