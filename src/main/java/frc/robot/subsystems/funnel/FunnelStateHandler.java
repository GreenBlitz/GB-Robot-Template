package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class FunnelStateHandler {

    private final Funnel funnel;

    public FunnelStateHandler(Robot robot){
        this.funnel = robot.getFunnel();
    }

    public Command setState(FunnelState state){
        return funnel.getCommandsBuilder().setPower(state.getPower());
    }

}
