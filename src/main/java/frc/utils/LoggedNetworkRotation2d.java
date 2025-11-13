// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class LoggedNetworkRotation2d extends LoggedNetworkInput {

	private final String key;
	private final DoubleEntry entry;
	private Rotation2d defaultValue = Rotation2d.fromRotations(0);
	private Rotation2d value;

	public LoggedNetworkRotation2d(String key) {
		this.key = key;
		this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0);
		this.value = defaultValue;
		Logger.registerDashboardInput(this);
	}

	public LoggedNetworkRotation2d(String key, Rotation2d defaultValue) {
		this(key);
		setDefault(defaultValue);
		this.value = defaultValue;
	}

	public void setDefault(Rotation2d defaultValue) {
		this.defaultValue = defaultValue;
		entry.set(entry.get(defaultValue.getRadians()));
	}

	public void set(Rotation2d value) {
		entry.set(value.getRadians());
	}

	public Rotation2d get() {
		return value;
	}

	private final LoggableInputs inputs = new LoggableInputs() {

		public void toLog(LogTable table) {
			table.put(removeSlash(key), value);
		}

		public void fromLog(LogTable table) {
			value = table.get(removeSlash(key), defaultValue);
		}

	};

	public void periodic() {
		if (!Logger.hasReplaySource()) {
			value = Rotation2d.fromRadians(entry.get());
		}
		Logger.processInputs(prefix, inputs);
	}

}
