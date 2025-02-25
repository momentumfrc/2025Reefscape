package frc.robot.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.encoder.MoEncoder;
import java.util.function.Consumer;

public class MoSparkMaxPID<Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> {
    protected final Type type;
    protected final SparkBase motorController;
    protected final ClosedLoopSlot pidSlot;
    protected final SparkClosedLoopController pidController;
    protected final Consumer<Consumer<SparkBaseConfig>> configurator;

    protected final MoEncoder<Dim, VDim> internalEncoder;

    /**
     * The last position or velocity setpoint measured in internal encoder units.
     */
    protected double lastSetpoint;

    /**
     * Constructs a MoSparkMaxPID
     * <p>
     * Note: the controller's internal encoder should be scaled to return the mechanism's position in units of rotation.
     * <p>
     * Note: we need the MoEncoder because it is solely responsible for keeping track of the encoder's internal units,
     * which are needed to calculate the units for the setpoints.
     *
     * @param type the type of PID controller
     * @param controller the motor controller
     * @param pidSlot the slot in which to save the PID constants
     */
    public MoSparkMaxPID(
            Type type,
            SparkBase controller,
            ClosedLoopSlot pidSlot,
            MoEncoder<Dim, VDim> internalEncoder,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        this.type = type;
        this.motorController = controller;
        this.pidSlot = pidSlot;
        this.pidController = motorController.getClosedLoopController();
        this.internalEncoder = internalEncoder;
        this.configurator = configurator;
    }

    public MoSparkMaxPID(
            Type type, SparkBase controller, ClosedLoopSlot pidSlot, MoEncoder<Dim, VDim> internalEncoder) {
        this(type, controller, pidSlot, internalEncoder, MoSparkConfigurator.forSparkBase(controller));
    }

    public Type getType() {
        return type;
    }

    public ClosedLoopSlot getPidSlot() {
        return pidSlot;
    }

    public void setP(double kP) {
        setConfigOption(config -> config.closedLoop.p(kP, pidSlot));
    }

    public void setI(double kI) {
        setConfigOption(config -> config.closedLoop.i(kI, pidSlot));
    }

    public void setD(double kD) {
        setConfigOption(config -> config.closedLoop.d(kD, pidSlot));
    }

    public void setFF(double kFF) {
        setConfigOption(config -> config.closedLoop.velocityFF(kFF, pidSlot));
    }

    public void setIZone(double iZone) {
        setConfigOption(config -> config.closedLoop.iZone(iZone, pidSlot));
    }

    public void setConfigOption(Consumer<SparkBaseConfig> op) {
        configurator.accept(op);
    }

    public double getLastOutput() {
        return this.motorController.get();
    }

    /**
     * Get the last position or velocity reference set on this PID controller, measured in internal encoder units.
     * @return The last setpoint of this controller
     */
    public double getSetpoint() {
        return lastSetpoint;
    }

    /**
     * Get the last position or velocity measurement of the mechanism controlled by this PID controller, measured in
     * internal encoder units.
     * @return The last measurement of the mechanism under control
     */
    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return internalEncoder.getPositionInEncoderUnits();
            case VELOCITY:
            case SMARTVELOCITY:
                return internalEncoder.getVelocityInEncoderUnits();
        }

        return 0;
    }

    public void setPositionReference(Measure<Dim> position) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }
        double value = position.in(internalEncoder.getInternalEncoderUnits());
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastSetpoint = value;
    }

    public void setVelocityReference(Measure<VDim> velocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }
        double value = velocity.in(internalEncoder.getInternalEncoderVelocityUnits());
        pidController.setReference(value, this.type.innerType, pidSlot);
        lastSetpoint = value;
    }

    public enum Type {
        POSITION(SparkBase.ControlType.kPosition),
        SMARTMOTION(SparkBase.ControlType.kMAXMotionPositionControl),
        VELOCITY(SparkBase.ControlType.kVelocity),
        SMARTVELOCITY(SparkBase.ControlType.kMAXMotionVelocityControl);

        public final SparkBase.ControlType innerType;

        private Type(SparkBase.ControlType innerType) {
            this.innerType = innerType;
        }
    }
}
