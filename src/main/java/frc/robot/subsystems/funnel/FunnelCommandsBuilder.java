package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelCommandsBuilder {

    private final Funnel funnel;

    public FunnelCommandsBuilder(Funnel funnel) {
        this.funnel = funnel;
    }

    //@formatter:off
    public Command setPower(double power) {
        return new FunctionalCommand(
                () -> {},
                () -> funnel.setPower(power),
                (interrupted) -> funnel.stop(),
                () -> false,
                funnel
        ).withName("Set power to: " + power);
    }
    //@formatter:on

    public Command stop() {
        return new InstantCommand(funnel::stop, funnel).withName("Stop");
    }

}
