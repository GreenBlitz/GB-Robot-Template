package frc.robot.hardware.motor.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.PIDAble;
import frc.robot.hardware.motor.ProfileAble;
import frc.robot.hardware.request.angle.IAngleRequest;
import frc.robot.hardware.request.value.IValueRequest;
import frc.robot.hardware.request.value.TalonFXValueRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.Phoenix6SignalBuilder;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.devicewrappers.TalonFXWrapper;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class TalonFXMotor implements IMotor, PIDAble, ProfileAble {

	protected final TalonFXWrapper motor;
	protected final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;
	protected final String logPath;
	private final ConnectedInputAutoLogged connectionInput;

	public TalonFXMotor(TalonFXWrapper motor, SysIdRoutine.Config sysidConfig, String logPath) {
		this.motor = motor;
		this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, true);
		this.logPath = logPath;
		this.connectionInput = new ConnectedInputAutoLogged();
		motor.optimizeBusUtilization();
	}

	public void fetchSignals(InputSignal... signals) {
		ArrayList<BaseStatusSignal> signalsToRefresh = new ArrayList<>(signals.length);

		for (InputSignal inputSignal : signals) {
			if (inputSignal instanceof Phoenix6SignalBuilder.Phoenix6Signal phoenix6Signal) {
				signalsToRefresh.add(phoenix6Signal.getStatusSignal());
			} else if (inputSignal instanceof Phoenix6SignalBuilder.Phoenix6LatencyBothSignal fxLatencyBothSignal) {
				// must be before FXLatencySignal because extends FXLatencySignal
				signalsToRefresh.add(fxLatencyBothSignal.getSlopeStatusSignal());
				signalsToRefresh.add(fxLatencyBothSignal.getStatusSignal());
			} else if (inputSignal instanceof Phoenix6SignalBuilder.Phoenix6LatencySignal phoenix6LatencySignal) {
				signalsToRefresh.add(phoenix6LatencySignal.getStatusSignal());
			}
		}

		connectionInput.connected = BaseStatusSignal.refreshAll(signalsToRefresh.toArray(new BaseStatusSignal[0])).isOK();
		Logger.processInputs(logPath, connectionInput);

		for (InputSignal inputSignal : signals) {
			Logger.processInputs(logPath, inputSignal);
		}
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return sysidConfigInfo;
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.setNeutralMode(neutralModeValue);
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.setPosition(position.getRotations());
	}


	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public void setPower(double power) {
		motor.set(power);
	}


	@Override
	public void setVoltage(IValueRequest voltageRequest) {
		motor.setControl(((TalonFXValueRequest) voltageRequest).getControlRequest());
	}

	@Override
	public void setTargetVelocity(IAngleRequest velocityRequest) {
		motor.setControl(((TalonFXValueRequest) velocityRequest).getControlRequest());
	}

	@Override
	public void setTargetPosition(IAngleRequest positionRequest) {
		motor.setControl(((TalonFXValueRequest) positionRequest).getControlRequest());
	}


	@Override
	public void setTargetProfiledVelocity(IAngleRequest profiledVelocityRequest) {
		motor.setControl(((TalonFXValueRequest) profiledVelocityRequest).getControlRequest());
	}

	@Override
	public void setTargetProfiledPosition(IAngleRequest profiledPositionRequest) {
		motor.setControl(((TalonFXValueRequest) profiledPositionRequest).getControlRequest());
	}

}
