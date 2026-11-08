package frc;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.InputSignal;
import org.littletonrobotics.junction.Logger;

public class CANCoderAlon implements IAngleEncoder {
    private final CANcoder canCoder;
    private final String logpath;
    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> supplyVoltage;
    public CANCoderAlon(int id, String logpath){
        canCoder = new CANcoder(id);
        this.logpath = logpath;
        this.position = canCoder.getPosition();
        this.supplyVoltage = canCoder.getSupplyVoltage();
    }

    public Rotation2d getPosition(){
        return Rotation2d.fromRadians(position.getValue().in(Units.Radians));
    }

    @Override
    public void setPosition(Rotation2d position) {
        canCoder.setPosition(position.getMeasure());
    }

    @Override
    public boolean isConnected() {
        return canCoder.isConnected();
    }

    public void logAll(){
        Logger.recordOutput(logpath+"/position",position.refresh().getValue());
        Logger.recordOutput(logpath+"/supply voltage",supplyVoltage.refresh().getValue());
    }

    @Override
    public void updateInputs(InputSignal<?>... inputSignals) {

    }

    public static void fuseCancoder(CANCoderAlon canCoder, AlonFX motor){
        motor.withFusedCancoder(canCoder.canCoder);
    }
}
