package frc.utils.beamBreaker.SimulationBeamBreaker;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

import java.util.function.Consumer;

public class SimulationBeamBreaker implements IBeamBreaker {


	public SendableChooser<Boolean> isObstructedSendableChooser;
	public boolean isObstructed = false;

	public SimulationBeamBreaker() {
		isObstructedSendableChooser = new SendableChooser<Boolean>();
		Consumer<Boolean> isObstructedConsumer = obstructed -> setIsObstructed(obstructed);
		isObstructedSendableChooser.onChange(isObstructedConsumer);
	}

	private void subsystemPeriodic() {
		isObstructed = isObstructedSendableChooser.getSelected();
	}

	public boolean getIsObstructed() {
		return isObstructed;
	}


	public void setIsObstructed(boolean isObstructed) {
		this.isObstructed = isObstructed;
	}


	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = getIsObstructed();
	}

}
