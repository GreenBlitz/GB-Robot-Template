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
        return switch (state){
            case PRE_SPEAKER -> getPreSpeakerCommand();
            case SPEAKER -> getSpeakerCommand();
            case PRE_AMP -> getPreAmpCommand();
            case AMP -> getAmpCommand();
            case ELEVATOR_TO_SHOOTER ->
        };
    }

    public Command getPreSpeakerCommand(){
        return new FunctionalCommand(
                () -> {},
                () -> funnel.getCommandsBuilder().setPower(FunnelState.PRE_SPEAKER.getPower()),
                interrupted -> funnel.getCommandsBuilder().stop(),
                funnel::isNoteInShooter
        );
    }

    public Command getSpeakerCommand(){
        return funnel.getCommandsBuilder().setPower(FunnelState.SPEAKER.getPower());
    }

    public Command getPreAmpCommand(){
        return new SequentialCommandGroup(
                new FunctionalCommand(
                        () -> {},
                        () -> funnel.getCommandsBuilder().setPower(FunnelState.PRE_SPEAKER.getPower()),
                        interrupted -> funnel.getCommandsBuilder().stop(),
                        funnel::isNoteInShooter
                ).withName("Move note to shooter"),
                new FunctionalCommand(
                        () -> {},
                        () -> funnel.getCommandsBuilder().setPower(FunnelState.PRE_AMP.getPower()),
                        interrupted -> funnel.getCommandsBuilder().stop(),
                        funnel::isNoteInElevator
                ).withName("Move note to elevator")
        );
    }

    public Command getAmpCommand(){
        return funnel.getCommandsBuilder().setPower(FunnelState.AMP.getPower());
    }

}
