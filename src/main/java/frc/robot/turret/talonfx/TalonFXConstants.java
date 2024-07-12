package frc.robot.turret.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXConstants {

    public static final TalonFXConfiguration TURRET_MOTOR_CONFIGURATION = new TalonFXConfiguration();

    static {
        TURRET_MOTOR_CONFIGURATION.Slot0.kP = 1;
        TURRET_MOTOR_CONFIGURATION.Slot0.kI = 0;
        TURRET_MOTOR_CONFIGURATION.Slot0.kD = 0;

        TURRET_MOTOR_CONFIGURATION.Feedback.RotorToSensorRatio = 50;

    }


}
