package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

public class PivotCommandsBuilder {

	private Pivot pivot;

	public PivotCommandsBuilder(Pivot pivot) {
		this.pivot = pivot;
	}

	public Command setPowers(double power) {
		return new FunctionalCommand(() -> {}, () -> pivot.setPower(power), (interrupted) -> pivot.stop(), () -> false, pivot);
	}

	public Command setPosition(Rotation2d position) {
		return new FunctionalCommand(() -> {}, () -> pivot.setTargetPosition(position), (interrupted) -> pivot.stop(), () -> false, pivot);
	}
	public Command setPosition (Supplier<Rotation2d> positionSupplier){
		return new FunctionalCommand(() -> {}, () -> pivot.setTargetPosition(positionSupplier.get()), (interrupted) -> pivot.stop(), () -> false, pivot);
	}

	public Command stop() {
		return new InstantCommand(() -> pivot.stop(), pivot);
	}

}
