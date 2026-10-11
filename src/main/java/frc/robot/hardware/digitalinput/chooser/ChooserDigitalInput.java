package frc.robot.hardware.digitalinput.chooser;

import frc.robot.hardware.digitalinput.DigitalInputInputs;
import frc.robot.hardware.digitalinput.IDigitalInput;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableRegistry;

public class ChooserDigitalInput implements IDigitalInput {

	private final Selectable<Boolean> digitalInputValueChooser;

	public ChooserDigitalInput(String chooserName) {
		this.digitalInputValueChooser = new Selectable<>();

		digitalInputValueChooser.addDefault("false", false);
		digitalInputValueChooser.add("true", true);
		TunableRegistry.putData(chooserName, digitalInputValueChooser);
	}

	@Override
	public void updateInputs(DigitalInputInputs inputs) {
		inputs.nonDebouncedValue = digitalInputValueChooser.getSelected();
		inputs.debouncedValue = inputs.nonDebouncedValue;
	}

}
