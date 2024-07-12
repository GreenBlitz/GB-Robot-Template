package frc.robot.turret.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

class TalonFXTurretConstants {

    protected static final boolean IS_FOC_ENABLED = true;
    protected static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration();

    static {
        MOTOR_CONFIGURATION.Slot0.kP = 1;
        MOTOR_CONFIGURATION.Slot0.kI = 0;
        MOTOR_CONFIGURATION.Slot0.kD = 0;

        MOTOR_CONFIGURATION.Feedback.RotorToSensorRatio = 50;
    }

}
