package frc.utils.costomUnits;

import java.nio.ByteBuffer;
import org.wpilib.util.struct.Struct;

public final class Angle2dStruct implements Struct<Angle2d> {
    @Override
    public Class<Angle2d> getTypeClass() {
        return Angle2d.class;
    }

    @Override
    public String getTypeName() {
        return "Rotation2d";
    }

    @Override
    public int getSize() {
        return DOUBLE_SIZE;
    }

    @Override
    public String getSchema() {
        return "double value";
    }

    @Override
    public Angle2d unpack(ByteBuffer bb) {
        double value = bb.getDouble();
        return new Angle2d(value);
    }

    @Override
    public void pack(ByteBuffer bb, Angle2d value) {
        bb.putDouble(value.getRadians());
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}

