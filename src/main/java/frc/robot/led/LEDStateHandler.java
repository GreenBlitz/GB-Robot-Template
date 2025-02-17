package frc.robot.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GBSubsystem;

public class LEDStateHandler extends GBSubsystem {
    public CANdle candle;

    public LEDStateHandler(String logPath,CANdle candle) {
		super(logPath);
		this.candle = candle;
    }

    public Command setState(LEDState state){
        Command command = new InstantCommand(() -> candle.animate(state.animation),this).ignoringDisable(true);
		return command;
    }


}
