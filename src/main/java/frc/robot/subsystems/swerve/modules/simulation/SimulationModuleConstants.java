package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulationModuleConstants {

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
