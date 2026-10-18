package frc.robot.hardware.digitalinput.chooser;

import frc.robot.hardware.digitalinput.DigitalInputInputs;
import frc.robot.hardware.digitalinput.IDigitalInput;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.Selectable;

public class ChooserDigitalInput implements IDigitalInput {

	private final Selectable<Boolean> digitalInputValueChooser;
    private final TelemetryTable telemetry;

	public ChooserDigitalInput(String chooserName) {
		this.digitalInputValueChooser = new Selectable<>();
        this.telemetry = Telemetry.getTable();
		digitalInputValueChooser.addDefault("false", false);
		digitalInputValueChooser.add("true", true);
		telemetry.log(chooserName, digitalInputValueChooser);
	}

    public ChooserDigitalInput(String chooserName, String tableName) {
		this.digitalInputValueChooser = new Selectable<>();
        this.telemetry = Telemetry.getTable(tableName);
		digitalInputValueChooser.addDefault("false", false);
		digitalInputValueChooser.add("true", true);
		telemetry.log(chooserName, digitalInputValueChooser);
	}

	@Override
	public void updateInputs(DigitalInputInputs inputs) {
		inputs.nonDebouncedValue = digitalInputValueChooser.getSelected();
		inputs.debouncedValue = inputs.nonDebouncedValue;
	}

}
