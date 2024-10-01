package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FunnelCommandsBuilder {

	private final Funnel funnel;

	public FunnelCommandsBuilder(Funnel funnel) {
		this.funnel = funnel;
	}

	//@formatter:off
    public Command setFunnelPower(double power) {
        return new FunctionalCommand(
                () -> funnel.setPower(power),
                () -> {},
                interrupted -> funnel.stop(),
                () -> false,
                funnel
        ).withName("Set funnel power: " + power);
    }

    public Command setFunnelPower(DoubleSupplier power) {
        return new FunctionalCommand(
                () -> {},
                () -> funnel.setPower(power.getAsDouble()),
                interrupted -> funnel.stop(),
                () -> false,
                funnel
        ).withName("Set funnel power by supplier");
    }
    //@formatter:on

	public Command stopFunnel() {
		return new RunCommand(funnel::stop, funnel).withName("Stop funnel");
	}

}
