package frc.utils.utilcommands;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * This Command logs a value or a supplier through logger
 */
public class LogCommand extends InstantCommand {

	public LogCommand(String key, String value) {
		super(() -> Logger.recordOutput(key, value));
	}

	public LogCommand(String key, boolean value) {
		super(() -> Logger.recordOutput(key, value));
	}

	public LogCommand(String key, int value) {
		super(() -> Logger.recordOutput(key, value));
	}

	public LogCommand(String key, double value) {
		super(() -> Logger.recordOutput(key, value));
	}

	public <T extends StructSerializable> LogCommand(String key, T value) {
		super(() -> Logger.recordOutput(key, value));
	}

	public <T extends WPISerializable> LogCommand(String key, T value) {
		super(() -> Logger.recordOutput(key, value));
	}

	interface IntegerSupplier extends Supplier<Integer>{}
	interface StringSupplier extends Supplier<String>{}
	interface TSupplier<T> extends Supplier<T> {}

	public LogCommand(String key, StringSupplier value) {
		super(() -> Logger.recordOutput(key, value.get()));
	}

	public LogCommand(String key, BooleanSupplier value) {
		super(() -> Logger.recordOutput(key, value.getAsBoolean()));
	}

	public LogCommand(String key, IntegerSupplier value) {
		super(() -> Logger.recordOutput(key, value.get()));
	}

	public LogCommand(String key, DoubleSupplier value) {
		super(() -> Logger.recordOutput(key, value.getAsDouble()));
	}

	public <T extends StructSerializable> LogCommand(String key, TSupplier<T> value) {
		super(() -> Logger.recordOutput(key, value.get()));
	}

	public <T extends WPISerializable> LogCommand(String key, Supplier<T> value) {
		super(() -> Logger.recordOutput(key, value.get()));
	}
}
