package frc.robot.turret.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SimulationConstants {
    public static double GEAR_RATIO = 50.0;
    public static int NUMBER_OF_MOTORS = 1;
    public static double JKG_METER_SQ = 0.00048;
    public static TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration();
    static {
        MOTOR_CONFIGURATION.Slot0.kP = 1;
    }
}
