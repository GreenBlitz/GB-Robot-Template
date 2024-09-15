package frc.robot.hardware.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6BothLatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;

public class CANCoderEncoder implements IAngleEncoder {

    private final CANcoder encoder;

    public CANCoderEncoder(CANcoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public void setPosition(Rotation2d position) {
        encoder.setPosition(position.getRotations());
    }

    @Override
    public boolean isOK() {
        return BaseStatusSignal.isAllGood(encoder.getPosition());
    }

    @Override
    public void updateInputs(ConnectedInputAutoLogged inputs) {
        inputs.connected = isOK();
    }

    @Override
    public void updateSignals(InputSignal... signals) {
        for (InputSignal signal : signals) {
            if (signal instanceof Phoenix6SignalBuilder.SignalGetter) {
                BaseStatusSignal.refreshAll(((Phoenix6SignalBuilder.SignalGetter) signal).getSignal());
            }
        }
    }

}
