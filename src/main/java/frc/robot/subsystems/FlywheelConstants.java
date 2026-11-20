package frc.robot.subsystems;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;
import frc.robot.subsystems.roller.VelocityRoller;
import frc.utils.angle2d.Angle2d;

public class FlywheelConstants {

    public static final double SENSOR_TO_MECHANISM_RATIO_MASTER = 0.6428;
    public static final double SENSOR_TO_MECHANISM_RATIO_FOLLOWER = 0.6428;

    public static final InvertedValue IS_MASTER_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue IS_FOLLOWER_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double kP = 0.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0.0735;
    public static final double kS = 0.24;
    public static final double kA = 0.012081;

    public static final double kP_SIM = 0.4;
    public static final double kI_SIM = 0;
    public static final double kD_SIM = 0;
    public static final double kS_SIM = 0.0;
    public static final double kV_SIM = 0.1385;
    public static final double kA_SIM = 0;

    public static final double MOMENT_OF_INERTIA = 0.01;
    public static final int CURRENT_LIMIT = 80;

    public static final Angle2d MAX_ACCELERATION = Angle2d.fromRotations(350);

    public static final Angle2d MIN_ERROR_TO_APPLY_BANG_BANG_CONTROL_RPS = Angle2d.fromRotations(3);
    public static final double MAX_BANG_BANG_POWER = 0.65;

    public static final Angle2d FLYWHEEL_ACCELERATION_THRESHOLD_FOR_SHOT = Angle2d.fromRotations(-105);

    private static final Slot0Configs configs = new Slot0Configs();

    private static final TalonFXFollowerConfig followerConfig = new TalonFXFollowerConfig();

    private static final SysIdRoutine.Config sysIdRoutine = new SysIdRoutine.Config();

    private static final FeedbackConfigs feed = new FeedbackConfigs();
    public static VelocityRoller createFlyWheel() {
        feed.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO_MASTER;
        configs.kA = kA;
        configs.kP = kP;
        configs.kV = kV;
        configs.kS = kS;
        return TalonFXRollerBuilder.buildVelocityRoller("Subsystems/FlyWheel", IDs.TalonFXIDs.FLY_WHEEL,configs,configs,followerConfig,sysIdRoutine,CURRENT_LIMIT,feed,MOMENT_OF_INERTIA,false,true);
    }

}
