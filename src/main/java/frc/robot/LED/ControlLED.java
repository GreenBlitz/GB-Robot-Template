package frc.robot.LED;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ControlLED extends Command {
    private Candle candle;
    public ControlLED(){
        this.candle = Candle.getInstance();
    }

    @Override
    public void execute() {
        candle.setColorAccordingToState(Robot.ROBOT_STATE);
    }

    @Override
    public void end(boolean interrupted) {
        candle.turnOff();
    }
}
