package frc.utils.beambreaker.simulationbeambreaker;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.beambreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beambreaker.IBeamBreaker;

import java.util.function.Consumer;

public class SimulationBeamBreaker implements IBeamBreaker {

	Consumer<Boolean> isObstructedConsumer = this::setIsObstructed;

	private boolean isObstructed;

	public SimulationBeamBreaker() {
		final SendableChooser<Boolean> isObstructedSendableChooser = new SendableChooser<>();
		this.isObstructed = SimulationBeamBreakerConstants.DEFAULT_STATE;
		isObstructedSendableChooser.onChange(isObstructedConsumer);
	}

	public void setIsObstructed(boolean isObstructed) {
		this.isObstructed = isObstructed;
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		isObstructedConsumer.accept(SmartDashboard.getBoolean("is obstructed", SimulationBeamBreakerConstants.DEFAULT_STATE));
		inputs.isObstructed = isObstructed;
	}

}
