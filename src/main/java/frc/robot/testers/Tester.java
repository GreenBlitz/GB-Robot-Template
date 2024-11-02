package frc.robot.testers;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
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

    TalonFXWrapper motor;
    TalonFXSimState simState;
    double gearRatio = 5;
    DCMotorSim physicsSimulation = new DCMotorSim(DCMotor.getFalcon500Foc(1), gearRatio, 0.001);

    public Tester(int id) {
        motor = new TalonFXWrapper(id);
        simState = motor.getSimState();
        StatusCode returnCode;

        TalonFXConfiguration fxCfg = new TalonFXConfiguration();
        fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        fxCfg.Feedback.SensorToMechanismRatio  = gearRatio;
        for (int i = 0; i < 5; ++i) {
            returnCode = motor.getConfigurator().apply(fxCfg);
            if (returnCode.isOK()) break;
        }

        BaseStatusSignal.setUpdateFrequencyForAll(100, motor.getPosition(), motor.getMotorVoltage(), motor.getVelocity());

        simState.Orientation = ChassisReference.CounterClockwise_Positive;
        simState.setSupplyVoltage(BatteryUtils.DEFAULT_VOLTAGE);


    }

    public void run() {
        physicsSimulation.setInputVoltage(simState.getMotorVoltage());
        physicsSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
        simState.setRawRotorPosition(Rotation2d.fromRadians(physicsSimulation.getAngularPositionRad()).times(gearRatio).getRotations());
        simState.setRotorVelocity(Rotation2d.fromRadians(physicsSimulation.getAngularVelocityRadPerSec()).times(gearRatio).getRotations());

        Logger.recordOutput("Test/"+ motor.getDeviceID()+"/VoltageSim",simState.getMotorVoltage());
        Logger.recordOutput("Test/"+ motor.getDeviceID()+"/VoltageMotor",motor.getMotorVoltage().getValue());
        Logger.recordOutput("Test/"+ motor.getDeviceID() +"/VelocityMotor",motor.getVelocity().getValue());
        Logger.recordOutput("Test/"+ motor.getDeviceID() +"/PositionMotor",motor.getPosition().getValue());
        Logger.recordOutput("Test/"+ motor.getDeviceID() +"/VelocityMech",Rotation2d.fromRadians(physicsSimulation.getAngularVelocityRadPerSec()).getRotations());
        Logger.recordOutput("Test/"+ motor.getDeviceID() +"/PositionMech",Rotation2d.fromRadians(physicsSimulation.getAngularPositionRad()).getRotations());
    }

    public void setControl(ControlRequest controlRequest) {
        motor.setControl(controlRequest);
    }

}
