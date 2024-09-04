package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.GBSubsystem;
import frc.utils.devicewrappers.SparkMaxConfiguration;
import frc.utils.devicewrappers.GBSparkMax;

public class sparkMaxSubsystem extends GBSubsystem {
    private final GBSparkMax motor;

    public sparkMaxSubsystem(String logPath) {
        super(logPath);
        motor = new GBSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);
        SparkMaxConfiguration config = new SparkMaxConfiguration();
        config.slot0.withP(1);
    }

    public void moveto(Rotation2d target){

    }



    @Override
    protected void subsystemPeriodic() {

    }
}
