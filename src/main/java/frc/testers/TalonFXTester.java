package frc.testers;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class TalonFXTester {

	public final TalonFXMotor talonFXMotor;
	private final Phoenix6LatencySignal position;
	private final Phoenix6AngleSignal velocity;
	private final Phoenix6DoubleSignal current;
	private final Phoenix6DoubleSignal voltage;

	private final Phoenix6FeedForwardRequest positionReq;
	private final Phoenix6FeedForwardRequest velocityReq;
	private final Phoenix6Request<Double> currentReq;
	private final Phoenix6Request<Double> voltageReq;

	public TalonFXTester(Phoenix6DeviceID phoenix6DeviceID) {
		final SimpleMotorSimulation sim = new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1), DCMotor.getKrakenX60Foc(1))
		);
		this.talonFXMotor = new TalonFXMotor("Test/TalonFX/" + phoenix6DeviceID.id(), phoenix6DeviceID, new SysIdRoutine.Config(), sim);

		final double freq = RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ;
		this.velocity = Phoenix6SignalBuilder.build(talonFXMotor.getDevice().getVelocity(), freq, AngleUnit.ROTATIONS);
		this.position = Phoenix6SignalBuilder.build(talonFXMotor.getDevice().getPosition(), velocity, freq, AngleUnit.ROTATIONS);
		this.current = Phoenix6SignalBuilder.build(talonFXMotor.getDevice().getTorqueCurrent(), freq);
		this.voltage = Phoenix6SignalBuilder.build(talonFXMotor.getDevice().getMotorVoltage(), freq);

		final boolean enableFOC = true;
		this.positionReq = Phoenix6RequestBuilder.build(new PositionVoltage(0), 0, enableFOC);
		this.velocityReq = Phoenix6RequestBuilder.build(new VelocityVoltage(0), 0, enableFOC);
		this.currentReq = Phoenix6RequestBuilder.build(new TorqueCurrentFOC(0));
		this.voltageReq = Phoenix6RequestBuilder.build(new VoltageOut(0), enableFOC);
	}

	public void update() {
		talonFXMotor.updateSimulation();
		talonFXMotor.updateInputs(position, velocity, current, voltage);
	}

	public void setTargetPosition(Rotation2d positionRot) {
		talonFXMotor.applyRequest(positionReq.withSetPoint(positionRot));
	}

	public void setTargetVelocity(Rotation2d velocityRPS) {
		talonFXMotor.applyRequest(velocityReq.withSetPoint(velocityRPS));
	}

	public void setTargetCurrent(double currentAmps) {
		talonFXMotor.applyRequest(currentReq.withSetPoint(currentAmps));
	}

	public void setTargetVoltage(double voltageVolts) {
		talonFXMotor.applyRequest(voltageReq.withSetPoint(voltageVolts));
	}

}
