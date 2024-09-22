package frc.robot.LED;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ControlLED extends Command {
    Candle candle = Candle.getInstance();
    public ControlLED(){}

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return Robot.ROBOT_STATE.equals(RobotStates.DEFAULT_STATE);
    }

    @Override
    public void end(boolean interrupted) {
        candle.turnOff();
    }
}
