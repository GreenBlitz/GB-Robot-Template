package frc.robot.testers;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.utils.battery.BatteryUtils;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

public class Tester implements ITester {

	TalonFX motor;
	TalonFXSimState simState;
	double gearRatio = 5;
	DCMotorSim physicsSimulation = new DCMotorSim(DCMotor.getFalcon500Foc(1), gearRatio, 0.001);

	StatusSignal<Double> positionSignal, voltageSignal, velocitySignal;

	public Tester(int id) {
		motor = new TalonFXWrapper(id);
		simState = motor.getSimState();
		StatusCode returnCode;

		TalonFXConfiguration fxCfg = new TalonFXConfiguration();
		fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		fxCfg.Feedback.SensorToMechanismRatio = gearRatio;
		for (int i = 0; i < 5; ++i) {
			returnCode = motor.getConfigurator().apply(fxCfg);
			if (returnCode.isOK())
				break;
		}

//        BaseStatusSignal.setUpdateFrequencyForAll(100, motor.getPosition(), motor.getMotorVoltage(), motor.getVelocity());
		positionSignal = motor.getPosition();
		positionSignal.setUpdateFrequency(1000);

		velocitySignal = motor.getVelocity();
		velocitySignal.setUpdateFrequency(1000);

		voltageSignal = motor.getMotorVoltage();
		voltageSignal.setUpdateFrequency(1000);

		simState.Orientation = ChassisReference.CounterClockwise_Positive;
		simState.setSupplyVoltage(BatteryUtils.DEFAULT_VOLTAGE);
	}

	public void run() {
		physicsSimulation.setInputVoltage(simState.getMotorVoltage());
		physicsSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
		simState.setRawRotorPosition(Rotation2d.fromRadians(physicsSimulation.getAngularPositionRad()).times(gearRatio).getRotations());
		simState.setRotorVelocity(Rotation2d.fromRadians(physicsSimulation.getAngularVelocityRadPerSec()).times(gearRatio).getRotations());

		System.out.println(BaseStatusSignal.refreshAll(voltageSignal, positionSignal, velocitySignal) + " " + motor.getDeviceID());

		Logger.recordOutput("Test/" + motor.getDeviceID() + "/VoltageSim", simState.getMotorVoltage());
		Logger.recordOutput("Test/" + motor.getDeviceID() + "/VoltageMotor", voltageSignal.getValue());
		Logger.recordOutput("Test/" + motor.getDeviceID() + "/VelocityMotor", velocitySignal.getValue());
		Logger.recordOutput("Test/" + motor.getDeviceID() + "/PositionMotor", positionSignal.getValue());
		Logger.recordOutput("Test/" + motor.getDeviceID() + "/PositionFreq", positionSignal.getAppliedUpdateFrequency());
		Logger.recordOutput(
			"Test/" + motor.getDeviceID() + "/VelocityMech",
			Rotation2d.fromRadians(physicsSimulation.getAngularVelocityRadPerSec()).getRotations()
		);
		Logger.recordOutput(
			"Test/" + motor.getDeviceID() + "/PositionMech",
			Rotation2d.fromRadians(physicsSimulation.getAngularPositionRad()).getRotations()
		);
	}

	public void setControl(ControlRequest controlRequest) {
		motor.setControl(controlRequest);
	}

}
