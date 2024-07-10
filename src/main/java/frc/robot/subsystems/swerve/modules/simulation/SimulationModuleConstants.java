package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.Conversions;

public class SimulationModuleConstants {

    protected static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;

    protected static final Rotation2d MAX_SPEED_PER_SECOND = Rotation2d.fromRotations(Conversions.distanceToRevolutions(
            SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            WHEEL_DIAMETER_METERS
    ));

    protected static final double DRIVE_GEAR_RATIO = 6.12;
    protected static final double STEER_GEAR_RATIO = (150.0 / 7.0);

    protected static final boolean ENABLE_FOC_DRIVE = true;
    protected static final boolean ENABLE_FOC_STEER = true;

    protected static final DCMotor DRIVE_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(1);
    protected static final DCMotor STEER_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(1);

    protected static final double DRIVE_MOMENT_OF_INERTIA = 0.003;
    protected static final double STEER_MOMENT_OF_INERTIA = 0.003;

    protected static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
        STEER_MOTOR_CONFIG.Slot0.kP = 72;
        STEER_MOTOR_CONFIG.Slot0.kI = 0;
        STEER_MOTOR_CONFIG.Slot0.kD = 0;
        STEER_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
    }
}
