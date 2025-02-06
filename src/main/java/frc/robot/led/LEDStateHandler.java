package frc.robot.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDStateHandler {
    public CANdle candle;

    public LEDStateHandler(CANdle candle) {
        this.candle = candle;
    }

    public Command setState(LEDState state){
        return new InstantCommand(() -> candle.animate(state.animation));
    }


}
