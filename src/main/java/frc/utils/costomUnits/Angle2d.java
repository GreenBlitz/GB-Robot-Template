package frc.utils.costomUnits;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.interpolation.Interpolatable;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.MathSharedStore;
import org.wpilib.math.util.Nat;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.protobuf.ProtobufSerializable;
import org.wpilib.util.struct.StructSerializable;

import java.util.Objects;

import static org.wpilib.units.Units.Radians;

public class Angle2d implements Interpolatable<Angle2d>, ProtobufSerializable, StructSerializable {
    public static final Angle2d ZERO = new Angle2d();

    /**
     * A preallocated Angle2d representing a clockwise rotation by π/2 rad (90°).
     *
     * <p>This exists to avoid allocations for common rotations.
     */
    public static final Angle2d CW_PI_2 = new Angle2d(-Math.PI / 2);

    /**
     * A preallocated Angle2d representing a clockwise rotation by 90° (π/2 rad).
     *
     * <p>This exists to avoid allocations for common rotations.
     */
    public static final Angle2d CW_90DEG = CW_PI_2;

    /**
     * A preallocated Angle2d representing a counterclockwise rotation by π/2 rad (90°).
     *
     * <p>This exists to avoid allocations for common rotations.
     */
    public static final Angle2d CCW_PI_2 = new Angle2d(Math.PI / 2);

    /**
     * A preallocated Angle2d representing a counterclockwise rotation by 90° (π/2 rad).
     *
     * <p>This exists to avoid allocations for common rotations.
     */
    public static final Angle2d CCW_90DEG = CCW_PI_2;

    /**
     * A preallocated Angle2d representing a counterclockwise rotation by π rad (180°).
     *
     * <p>This exists to avoid allocations for common rotations.
     */
    public static final Angle2d PI = new Angle2d(Math.PI);

    /**
     * A preallocated Angle2d representing a counterclockwise rotation by 180° (π rad).
     *
     * <p>This exists to avoid allocations for common rotations.
     */
    public static final Angle2d CCW_180DEG = PI;

    private final double m_value;
    private final double m_cos;
    private final double m_sin;

    /** Constructs an Angle2d with a default angle of 0 degrees. */
    public Angle2d() {
        m_value = 0.0;
        m_cos = 1.0;
        m_sin = 0.0;
    }

    /**
     * Constructs an Angle2d with the given radian value.
     *
     * @param value The value of the angle in radians.
     */
    public Angle2d(double value) {
        m_value = value;
        m_cos = Math.cos(value);
        m_sin = Math.sin(value);
    }

    /**
     * Constructs an Angle2d with the given x and y (cosine and sine) components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    public Angle2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            m_cos = x / magnitude;
            m_sin = y / magnitude;
        } else {
            m_cos = 1.0;
            m_sin = 0.0;
            MathSharedStore.reportError(
                    "x and y components of Angle2d are zero\n", Thread.currentThread().getStackTrace());
        }
        m_value = Math.atan2(m_sin, m_cos);
    }

    /**
     * Constructs an Angle2d with the given angle.
     *
     * @param angle The angle of the rotation.
     */
    public Angle2d (Angle angle) {
        this(angle.in(Radians));
    }

    /**
     * Constructs an Angle2d from a rotation matrix.
     *
     * @param rotationMatrix The rotation matrix.
     * @throws IllegalArgumentException if the rotation matrix isn't special orthogonal.
     */
    public Angle2d(Matrix<N2, N2> rotationMatrix) {
        final var R = rotationMatrix;

        // Require that the rotation matrix is special orthogonal. This is true if
        // the matrix is orthogonal (RRᵀ = I) and normalized (determinant is 1).
        if (R.times(R.transpose()).minus(Matrix.eye(Nat.N2())).normF() > 1e-9) {
            var msg = "Rotation matrix isn't orthogonal\n\nR =\n" + R.getStorage().toString() + '\n';
            MathSharedStore.reportError(msg, Thread.currentThread().getStackTrace());
            throw new IllegalArgumentException(msg);
        }
        if (Math.abs(R.det() - 1.0) > 1e-9) {
            var msg =
                    "Rotation matrix is orthogonal but not special orthogonal\n\nR =\n"
                            + R.getStorage().toString()
                            + '\n';
            MathSharedStore.reportError(msg, Thread.currentThread().getStackTrace());
            throw new IllegalArgumentException(msg);
        }

        // R = [cosθ  −sinθ]
        //     [sinθ   cosθ]
        m_cos = R.get(0, 0);
        m_sin = R.get(1, 0);

        m_value = Math.atan2(m_sin, m_cos);
    }

    /**
     * Constructs and returns an Angle2d with the given radian value.
     *
     * @param radians The value of the angle in radians.
     * @return The rotation object with the desired angle value.
     */
    public static Angle2d fromRadians(double radians) {
        return new Angle2d(radians);
    }

    /**
     * Constructs and returns an Angle2d with the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     * @return The rotation object with the desired angle value.
     */
    public static Angle2d fromDegrees(double degrees) {
        return new Angle2d(Math.toRadians(degrees));
    }

    /**
     * Constructs and returns an Angle2d with the given number of rotations.
     *
     * @param rotations The value of the angle in rotations.
     * @return The rotation object with the desired angle value.
     */
    public static Angle2d fromRotations(double rotations) {
        return new Angle2d(Units.rotationsToRadians(rotations));
    }

    /**
     * Constructs and returns an Angle2d with the given number of rotations.
     *
     * @param rotation2d The value of the angle in Rotation2d.
     * @return The rotation object with the desired angle value.
     */
    //use carefully, it does a Modulo 360
    public static Angle2d fromRotation2d(Rotation2d rotation2d){
        return Angle2d.fromRadians(rotation2d.getRadians());
    }

    /**
     * Adds two rotations together, with the result being bounded between -π and π.
     *
     * <p>For example, <code>Angle2d.fromDegrees(30).plus(Angle2d.fromDegrees(60))</code> equals
     * <code>Angle2d(Math.PI/2.0)</code>
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public Angle2d plus(Angle2d other) {
        return rotateBy(other);
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the new rotation.
     *
     * <p>For example, <code>Angle2d.fromDegrees(10).minus(Angle2d.fromDegrees(100))</code>
     * equals <code>Angle2d(-Math.PI/2.0)</code>
     *
     * @param other The rotation to subtract.
     * @return The difference between the two rotations.
     */
    public Angle2d minus(Angle2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Takes the inverse of the current rotation. This is simply the negative of the current angular
     * value.
     *
     * @return The inverse of the current rotation.
     */
    public Angle2d unaryMinus() {
        return new Angle2d(-m_value);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Angle2d.
     */
    public Angle2d times(double scalar) {
        return new Angle2d(m_value * scalar);
    }

    /**
     * Divides the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Angle2d.
     */
    public Angle2d div(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>The matrix multiplication is as follows:
     *
     * <pre>
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(sin_new, cos_new)
     * </pre>
     *
     * @param other The rotation to rotate by.
     * @return The new rotated Angle2d.
     */
    public Angle2d rotateBy(Angle2d other) {
        return new Angle2d(
                m_cos * other.m_cos - m_sin * other.m_sin, m_cos * other.m_sin + m_sin * other.m_cos);
    }

    /**
     * Returns the current rotation relative to the given rotation.
     *
     * @param other The rotation describing the orientation of the new coordinate frame that the
     *     current rotation will be converted into.
     * @return The current rotation relative to the new orientation of the coordinate frame.
     */
    public Angle2d relativeTo(Angle2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Returns matrix representation of this rotation.
     *
     * @return Matrix representation of this rotation.
     */
    public Matrix<N2, N2> toMatrix() {
        // R = [cosθ  −sinθ]
        //     [sinθ   cosθ]
        return MatBuilder.fill(Nat.N2(), Nat.N2(), m_cos, -m_sin, m_sin, m_cos);
    }

    public Rotation2d toRotation2d(){
        return Rotation2d.fromRadians(getRadians());
    }

    /**
     * Returns the measure of the Angle2d.
     *
     * @return The measure of the Angle2d.
     */
    public Angle getMeasure() {
        return Radians.of(getRadians());
    }

    /**
     * Returns the radian value of the Angle2d.
     *
     * @return The radian value of the Angle2d.
     */
    public double getRadians() {
        return m_value;
    }

    /**
     * Returns the degree value of the Angle2d.
     *
     * @return The degree value of the Angle2d.
     */
    public double getDegrees() {
        return Math.toDegrees(m_value);
    }

    /**
     * Returns the number of rotations of the Angle2d.
     *
     * @return The number of rotations of the Angle2d.
     */
    public double getRotations() {
        return Units.radiansToRotations(m_value);
    }

    /**
     * Returns the cosine of the Angle2d.
     *
     * @return The cosine of the Angle2d.
     */
    public double getCos() {
        return m_cos;
    }

    /**
     * Returns the sine of the Angle2d.
     *
     * @return The sine of the Angle2d.
     */
    public double getSin() {
        return m_sin;
    }

    /**
     * Returns the tangent of the Angle2d.
     *
     * @return The tangent of the Angle2d.
     */
    public double getTan() {
        return m_sin / m_cos;
    }

    @Override
    public String toString() {
        return String.format("Angle2d(Rads: %.2f, Deg: %.2f)", m_value, Math.toDegrees(m_value));
    }

    /**
     * Checks equality between this Angle2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        return obj instanceof Angle2d other
                && Math.hypot(m_cos - other.m_cos, m_sin - other.m_sin) < 1E-9;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_value);
    }

    @Override
    public Angle2d interpolate(Angle2d endValue, double t) {
        return plus(endValue.minus(this).times(Math.clamp(t, 0, 1)));
    }


    /** Angle2d protobuf for serialization. */
    public static final Angle2dProto proto = new Angle2dProto();

    /** Angle2d struct for serialization. */
    public static final Angle2dStruct struct = new Angle2dStruct();
}
