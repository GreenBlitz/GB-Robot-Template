package frc.utils.costomUnits;

import org.wpilib.math.geometry.proto.detail.ProtobufRotation2d;
import org.wpilib.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public final class Angle2dProto implements Protobuf<Angle2d, ProtobufRotation2d> {
    @Override
    public Class<Angle2d> getTypeClass() {
        return Angle2d.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufRotation2d.getDescriptor();
    }

    @Override
    public ProtobufRotation2d createMessage() {
        return ProtobufRotation2d.newInstance();
    }

    @Override
    public Angle2d unpack(ProtobufRotation2d msg) {
        return new Angle2d(msg.getValue());
    }

    @Override
    public void pack(ProtobufRotation2d msg, Angle2d value) {
        msg.setValue(value.getRadians());
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}

