package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.utils.utilcommands.InitExecuteCommand;
import frc.utils.utilcommands.LoggedDashboardCommand;

import java.util.function.DoubleSupplier;

public class ElbowCommandsBuilder {

	private final Elbow elbow;

	public ElbowCommandsBuilder(Elbow elbow) {
		this.elbow = elbow;
	}

	//@formatter:off
	public Command moveToAngle(Rotation2d angle) {
		return new FunctionalCommand(
			() -> elbow.setTargetAngle(angle),
			() -> {},
			interrupted -> elbow.stayInPlace(),
			() -> false,
			elbow
		).withName("Move to angle: " + angle);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return new FunctionalCommand(
			() -> {},
			() -> elbow.setPower(powerSupplier.getAsDouble()),
			interrupted -> elbow.stayInPlace(),
			() -> false,
			elbow
		).withName("Set power");
	}
	//@formatter:on

	public Command stayInPlace() {
		return new InitExecuteCommand(() -> {}, elbow::stayInPlace, elbow).withName("Stay in place");
	}

	public Command voltageControlByDashboard(String widgetName) {
		return new LoggedDashboardCommand(widgetName, elbow::setVoltage, elbow).withName("Voltage by dashboard");
	}

}
