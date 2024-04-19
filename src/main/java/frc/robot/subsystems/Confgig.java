package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Confgig {

    protected static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
        STEER_MOTOR_CONFIG.Slot0.kP = 50;
        STEER_MOTOR_CONFIG.Slot0.kI = 0;
        STEER_MOTOR_CONFIG.Slot0.kD = 0;
        STEER_MOTOR_CONFIG.Slot1.kP = 2.5;
        STEER_MOTOR_CONFIG.Slot1.kI = 0;
        STEER_MOTOR_CONFIG.Slot1.kD = 0;
    }
}
