package frc.robot.hardware.talonfx;

import com.ctre.phoenix6.StatusSignal;

public record TalonFXSignals(
        StatusSignal<Double> position,
        StatusSignal<Double> velocity,
        StatusSignal<Double> current,
        StatusSignal<Double> voltage
) {
}
