package frc.testers;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.InputSignal;
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
    public final Phoenix6LatencySignal position;
    public final Phoenix6AngleSignal velocity;
    public final Phoenix6DoubleSignal current;
    public final Phoenix6DoubleSignal voltage;

    public final Phoenix6FeedForwardRequest positionReq;
    public final Phoenix6FeedForwardRequest velocityReq;
    public final Phoenix6Request<Double> currentReq;
    public final Phoenix6Request<Double> voltageReq;

    public TalonFXTester(Phoenix6DeviceID phoenix6DeviceID) {
        this.talonFXMotor = new TalonFXMotor("Test/TalonFX/" + phoenix6DeviceID.id(), phoenix6DeviceID, new SysIdRoutine.Config());

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

    public void resetPosition(Rotation2d position) {
        talonFXMotor.resetPosition(position);
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
