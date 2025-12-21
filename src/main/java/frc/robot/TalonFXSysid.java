package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXWrapper;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class TalonFXSysid extends GBSubsystem {

    private final TalonFXWrapper motor;
    private final SysIdCalibrator sysIdCalibrator;

    public TalonFXSysid(Phoenix6DeviceID deviceID, double rampRate, double stepVoltage) {
        super("TalonFXSysid/" + deviceID.id());
        SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(
                buildSysidConfig(rampRate, stepVoltage),
                true);
        this.motor = new TalonFXWrapper(deviceID);
        this.sysIdCalibrator = new SysIdCalibrator(
                sysIdConfigInfo,
                this,
                motor::setVoltage
        );

        /* Speed up signals for better characterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage());

        /* Optimize out the other signals, since they're not useful for SysId */
        motor.optimizeBusUtilization();

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.Feedback.SensorToMechanismRatio = (63.0 / 17.0);

        motor.applyConfiguration(configuration);


        TalonFXWrapper follower = new TalonFXWrapper(new Phoenix6DeviceID(11, BusChain.SUPERSTRUCTURE));

        TalonFXConfiguration configuration2 = new TalonFXConfiguration();
        configuration2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration2.Feedback.SensorToMechanismRatio = (63.0 / 17.0);
        follower.applyConfiguration(configuration);

        follower.setControl(new Follower(deviceID.id(), false));

        motor.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);
    }

    private SysIdRoutine.Config buildSysidConfig(double rampRate, double stepVoltage) {
        return new SysIdRoutine.Config(
                Units.Volts.of(rampRate).per(Units.Second),
                Units.Volts.of(stepVoltage),
                null,
                state -> SignalLogger.writeString(getLogPath() + "/state", state.toString())
        );
    }

    public SysIdCalibrator getSysIdCalibrator() {
        return sysIdCalibrator;
    }
}
