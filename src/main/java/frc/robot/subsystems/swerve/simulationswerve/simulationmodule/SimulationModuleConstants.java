package frc.robot.subsystems.swerve.simulationswerve.simulationmodule;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulationModuleConstants {

    protected static final double DRIVE_MOMENT_OF_INERTIA = 0.003;

    protected static final double STEER_MOMENT_OF_INERTIA = 0.003;

    protected static final DCMotor DRIVE_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(1);

    protected static final DCMotor STEER_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(1);

    protected static final double STEER_MOTOR_P = 72;

    protected static final double STEER_MOTOR_I = 0;

    protected static final double STEER_MOTOR_D = 0;

    protected static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
        STEER_MOTOR_CONFIG.Slot0.kP = SimulationModuleConstants.STEER_MOTOR_P;
        STEER_MOTOR_CONFIG.Slot0.kI = SimulationModuleConstants.STEER_MOTOR_I;
        STEER_MOTOR_CONFIG.Slot0.kD = SimulationModuleConstants.STEER_MOTOR_D;
        STEER_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
    }
}
