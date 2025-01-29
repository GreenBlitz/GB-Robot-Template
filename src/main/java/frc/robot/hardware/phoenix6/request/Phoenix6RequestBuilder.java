package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

public class Phoenix6RequestBuilder {

	public static Phoenix6Request<Double> build(VoltageOut voltageOut) {
		return new Phoenix6Request<>(voltageOut.Output, voltageOut, voltageOut::withOutput);
	}

	public static Phoenix6Request<Double> build(TorqueCurrentFOC torqueCurrentFOC) {
		return new Phoenix6Request<>(torqueCurrentFOC.Output, torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

}
