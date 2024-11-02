package frc.robot.testers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.hardware.motor.phoenix6.SimpleWheelSimulation;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.signal.phoenix6.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix6.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix6.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

public class CoolTester implements ITester {

	TalonFXMotor motor;
	double gearRatio = 5;
	SimpleWheelSimulation mechanismSimulation = new SimpleWheelSimulation(DCMotor.getFalcon500Foc(1), gearRatio, 0.001);

	Phoenix6DoubleSignal voltageSignal;
	Phoenix6AngleSignal positionSignal;

	public CoolTester(int id) {
		TalonFXConfiguration fxCfg = new TalonFXConfiguration();
		fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		fxCfg.Feedback.SensorToMechanismRatio = gearRatio;
		motor = new TalonFXMotor("Test/" + id + "/", new Phoenix6DeviceID(id), fxCfg, mechanismSimulation);


		voltageSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getMotor().getMotorVoltage(), 1000);
		positionSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getMotor().getPosition(), 1000, AngleUnit.ROTATIONS);
//		BaseStatusSignal.setUpdateFrequencyForAll(100, motor.getMotor().getPosition(), motor.getMotor().getMotorVoltage());
	}

	public void run() {
		motor.updateSimulation();
        motor.updateSignals(voltageSignal, positionSignal);

//		Logger.recordOutput(
//			"Test/" + motor.getMotor().getDeviceID() + "/Refresh",
//			BaseStatusSignal.refreshAll(positionSignal.getSignal(), voltageSignal.getSignal()).isOK()
//		);
		Logger.recordOutput("Test/" + motor.getMotor().getDeviceID() + "/VoltageSignal", voltageSignal.getLatestValue());
		Logger.recordOutput(
			"Test/" + motor.getMotor().getDeviceID() + "/Mahaha",
			motor.getTalonFXSimulation().getMotorSimState().getMotorVoltage()
		);
		Logger.recordOutput("Test/" + motor.getMotor().getDeviceID() + "/PositionSignal", positionSignal.getLatestValue().getRotations());

		Logger.recordOutput(
			"Test/" + motor.getMotor().getDeviceID() + "/VelocityMech",
			mechanismSimulation.getSystemVelocityRotationsPerSecond().getRotations()
		);
		Logger.recordOutput(
			"Test/" + motor.getMotor().getDeviceID() + "/PositionMech",
			mechanismSimulation.getSystemPositionRotations().getRotations()
		);
	}

	public void setControl(ControlRequest controlRequest) {
		motor.getMotor().setControl(controlRequest);
	}

}
