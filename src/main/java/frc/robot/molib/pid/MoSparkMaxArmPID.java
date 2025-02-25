package frc.robot.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.encoder.MoRotationEncoder;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Subclass of a {@link MoSparkMaxPID} that uses an ArmFeedForward to counteract gravity, linearizing the system response.
 */
public class MoSparkMaxArmPID extends MoSparkMaxPID<AngleUnit, AngularVelocityUnit> {
    private Optional<ArmFeedforward> armFF = Optional.empty();

    private Supplier<Measure<AngleUnit>> getAngleFromHorizontal;

    private double kS = 0;
    private double kG = 0;
    private double kV = 0;

    private double lastFF;

    /**
     * Constructs a MoSparkMaxArmPID.
     * <p>
     * Note: the controller's internal encoder should be scaled to return the arm's position in units of rotation.
     *
     * @param type the type of PID controller
     * @param controller the motor controller
     * @param pidSlot the slot in which to save the PID constants
     * @param getAngleFromHorizontal A supplier that provides the current angle of the arm. This angle should be measured from the
     *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor)
     */
    public MoSparkMaxArmPID(
            Type type,
            SparkBase controller,
            ClosedLoopSlot pidSlot,
            MoRotationEncoder encoder,
            Supplier<Measure<AngleUnit>> getAngleFromHorizontal,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        super(type, controller, pidSlot, encoder, configurator);
        this.getAngleFromHorizontal = getAngleFromHorizontal;
    }

    public MoSparkMaxArmPID(
            Type type,
            SparkBase controller,
            ClosedLoopSlot pidSlot,
            MoRotationEncoder encoder,
            Supplier<Measure<AngleUnit>> getAngleFromHorizontal) {
        this(type, controller, pidSlot, encoder, getAngleFromHorizontal, MoSparkConfigurator.forSparkBase(controller));
    }

    public void setKS(double kS) {
        this.kS = kS;
        this.armFF = Optional.empty();
    }

    public void setKG(double kG) {
        this.kG = kG;
        this.armFF = Optional.empty();
    }

    public void setKV(double kV) {
        this.kV = kV;
        this.armFF = Optional.empty();
    }

    public double getLastFF() {
        return lastFF;
    }

    private double getFF(Measure<AngleUnit> position, Measure<AngularVelocityUnit> velocity) {
        if (armFF.isEmpty()) {
            this.armFF = Optional.of(new ArmFeedforward(kS, kG, kV));
        }

        // The position must be in Units.Radians, because the ArmFeedForward just passes the value into
        // Math.cos() which expects inputs in radians.
        // However, the required velocity units depend on the units of kV. Since we tend to use rotations on our
        // arms, sysid returns kV in units of v*s/rot, so we need velocity in units of rot/s.
        return this.armFF.get().calculate(position.in(Units.Radians), velocity.in(Units.RotationsPerSecond));
    }

    public void setPositionReference(Measure<AngleUnit> desiredPosition) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }

        double ff = getFF(this.getAngleFromHorizontal.get(), Units.RotationsPerSecond.zero());
        double value = desiredPosition.in(internalEncoder.getInternalEncoderUnits());

        this.pidController.setReference(
                value, this.type.innerType, pidSlot, ff, SparkClosedLoopController.ArbFFUnits.kVoltage);
        this.lastFF = ff;
        this.lastSetpoint = value;
    }

    public void setVelocityReference(Measure<AngularVelocityUnit> desiredVelocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }

        double ff = -1 * getFF(this.getAngleFromHorizontal.get(), Units.RotationsPerSecond.zero());

        double value_internal = desiredVelocity.in(internalEncoder.getInternalEncoderVelocityUnits());
        this.pidController.setReference(
                value_internal, this.type.innerType, pidSlot, ff, SparkClosedLoopController.ArbFFUnits.kVoltage);
        this.lastFF = ff;
        this.lastSetpoint = value_internal;
    }
}
