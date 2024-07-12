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

import static frc.robot.turret.talonfx.TalonFXTurretConstants.IS_FOC_ENABLED;

public class TalonFXTurret implements ITurret {

    private final TalonFXWrapper motor;

    private final VelocityDutyCycle velocityDutyCycle;
    private final PositionDutyCycle positionDutyCycle;

    private final StatusSignal<Double> positionSignal;
    private final StatusSignal<Double> velocitySignal;
    private final StatusSignal<Double> accelerationSignal;
    private final StatusSignal<Double> currentSignal;
    private final StatusSignal<Double> voltageSignal;

    public TalonFXTurret(CTREDeviceID motorID, TalonFXConfiguration motorConfiguration) {
        this.motor = new TalonFXWrapper(motorID);
        motor.applyConfiguration(motorConfiguration);

        this.positionDutyCycle = new PositionDutyCycle(0).withEnableFOC(IS_FOC_ENABLED);
        this.velocityDutyCycle = new VelocityDutyCycle(0).withEnableFOC(IS_FOC_ENABLED);

        this.positionSignal = motor.getPosition();
        this.velocitySignal = motor.getVelocity();
        this.accelerationSignal = motor.getAcceleration();
        this.currentSignal = motor.getStatorCurrent();
        this.voltageSignal = motor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.DEFAULT_CAN_FREQUENCY,
                positionSignal,
                accelerationSignal,
                velocitySignal,
                voltageSignal,
                currentSignal
        );

        motor.optimizeBusUtilization();
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
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(velocitySignal, voltageSignal, currentSignal).isOK();

        inputs.position = Rotation2d.fromRotations(motor.getLatencyCompensatedPosition());
        inputs.velocity = Rotation2d.fromRotations(motor.getLatencyCompensatedVelocity());
        inputs.outputCurrent = currentSignal.getValue();
        inputs.outputVoltage = voltageSignal.getValue();
    }
}
