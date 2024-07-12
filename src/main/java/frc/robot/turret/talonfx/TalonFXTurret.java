package frc.robot.turret.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.GlobalConstants;
import frc.robot.turret.ITurret;
import frc.robot.turret.TurretInputsAutoLogged;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXTurret implements ITurret {
    private final TalonFXWrapper motor;
    private final VelocityDutyCycle velocityDutyCycle;
    private final PositionDutyCycle positionDutyCycle;


    private final StatusSignal<Double> velocitySignal;
    private final StatusSignal<Double> voltageSignal;
    private final StatusSignal<Double> currentSignal;

    public TalonFXTurret(CTREDeviceID turretID, TalonFXConfiguration motorConfiguration) {
        this.motor = new TalonFXWrapper(turretID);
        this.motor.applyConfiguration(motorConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.DEFAULT_CAN_FREQUENCY);

        motor.optimizeBusUtilization();

        this.positionDutyCycle = new PositionDutyCycle(0);
        this.velocityDutyCycle = new VelocityDutyCycle(0);

        this.velocitySignal = motor.getVelocity();
        this.voltageSignal = motor.getMotorVoltage();
        this.currentSignal = motor.getStatorCurrent();
    }

    @Override
    public void setVelocity(double velocity) {
        motor.setControl(velocityDutyCycle.withVelocity(velocity));
    }

    @Override
    public void setPosition(Rotation2d angle) {
        motor.setControl(positionDutyCycle.withPosition(angle.getRotations()));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(velocitySignal, voltageSignal, currentSignal);

        inputs.position = Rotation2d.fromRotations(motor.getLatencyCompensatedPosition());
        inputs.velocity = Rotation2d.fromRotations(velocitySignal.getValue());
        inputs.outputCurrent = currentSignal.getValue();
        inputs.outputVoltage = voltageSignal.getValue();
    }
}
