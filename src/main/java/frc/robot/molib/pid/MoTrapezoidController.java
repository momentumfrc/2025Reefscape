package frc.robot.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.molib.encoder.MoEncoder;
import java.util.function.Consumer;

public abstract class MoTrapezoidController<
        Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>, ADim extends PerUnit<VDim, TimeUnit>> {

    private static final double kDt = 0.02;

    private final SparkClosedLoopController pidController;
    private final MoEncoder<Dim, VDim> encoder;
    private final ClosedLoopSlot pidSlot;
    private final Consumer<Consumer<SparkBaseConfig>> configurator;

    private double maxVel, maxAccel;
    private TrapezoidProfile profile;

    private final Dim posUnit;
    private final VDim velUnit;

    private MutableMeasure<Dim, ?, ?> posSetpoint;
    private MutableMeasure<VDim, ?, ?> velSetpoint;
    private MutableMeasure<VDim, ?, ?> nextVelSetpoint;

    private double lastFF;
    private double lastReference;

    private MutableMeasure<Dim, ?, ?> errorZone;

    private IntegerLogEntry loopTimeLog;

    @SuppressWarnings("unchecked")
    public MoTrapezoidController(
            String name,
            SparkClosedLoopController pidController,
            ClosedLoopSlot pidSlot,
            MoEncoder<Dim, VDim> encoder,
            Consumer<Consumer<SparkBaseConfig>> configurator,
            Dim posUnit,
            VDim velUnit,
            ADim accelUnit) {
        this.pidController = pidController;
        this.encoder = encoder;
        this.pidSlot = pidSlot;
        this.configurator = configurator;

        this.posUnit = posUnit;
        this.velUnit = velUnit;

        posSetpoint = (MutableMeasure<Dim, ?, ?>) posUnit.mutable(0);
        velSetpoint = (MutableMeasure<VDim, ?, ?>) velUnit.mutable(0);
        nextVelSetpoint = (MutableMeasure<VDim, ?, ?>) velUnit.mutable(0);

        configurator.accept(config -> config.closedLoop.velocityFF(0));

        errorZone = (MutableMeasure<Dim, ?, ?>) posUnit.mutable(0);

        loopTimeLog = new IntegerLogEntry(DataLogManager.getLog(), "/Time/Trapezoid/" + name);
    }

    public double getLastFF() {
        return lastFF;
    }

    public double getLastReference() {
        return lastReference;
    }

    public double getLastMeasurement() {
        return encoder.getPosition().in(this.posUnit);
    }

    public void setP(double kP) {
        configurator.accept(config -> config.closedLoop.p(kP, pidSlot));
    }

    public void setI(double kI) {
        configurator.accept(config -> config.closedLoop.i(kI, pidSlot));
    }

    public void setD(double kD) {
        configurator.accept(config -> config.closedLoop.d(kD, pidSlot));
    }

    public void setIZone(double iZone) {
        configurator.accept(config -> config.closedLoop.iZone(iZone, pidSlot));
    }

    public void setDFilter(double dFilter) {
        configurator.accept(config -> config.closedLoop.dFilter(dFilter, pidSlot));
    }

    public void setIMaxAccum(double iMaxAccum) {
        configurator.accept(config -> config.closedLoop.iMaxAccum(iMaxAccum, pidSlot));
    }

    public void setMaxVelocity(double maxVel) {
        this.maxVel = maxVel;
        this.profile = null;
    }

    public void setMaxAcceleration(double maxAccel) {
        this.maxAccel = maxAccel;
        this.profile = null;
    }

    public void setErrorZone(double errorZone) {
        this.errorZone.mut_replace(errorZone, posUnit);
    }

    protected abstract double getFF(Measure<VDim> velSetpoint, Measure<VDim> nextVelSetpoint);

    public void setPositionReference(Measure<Dim> reference) {
        long time = WPIUtilJNI.getSystemTime();

        if (!encoder.getPosition().isNear(reference, errorZone)) {
            if (this.profile == null) {
                this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVel, maxAccel));
            }

            TrapezoidProfile.State currentState = new TrapezoidProfile.State(
                    encoder.getPosition().in(posUnit), encoder.getVelocity().in(velUnit));
            TrapezoidProfile.State goalState = new TrapezoidProfile.State(reference.in(posUnit), 0);

            TrapezoidProfile.State setpoint = profile.calculate(kDt, currentState, goalState);

            posSetpoint.mut_replace(setpoint.position, posUnit);
            velSetpoint.mut_replace(setpoint.velocity, velUnit);

            TrapezoidProfile.State nextSetpoint = profile.calculate(kDt * 2, currentState, goalState);
            nextVelSetpoint.mut_replace(nextSetpoint.velocity, velUnit);
        } else {
            // TODO: figure out the generics so I can skip the intermediate unitless value here
            posSetpoint.mut_replace(reference.in(posUnit), posUnit);
            velSetpoint.mut_replace(0, velUnit);
            nextVelSetpoint.mut_replace(0, velUnit);
        }

        double ff = getFF(velSetpoint, nextVelSetpoint);
        double value = posSetpoint.in(encoder.getInternalEncoderUnits());

        this.pidController.setReference(value, SparkBase.ControlType.kPosition, pidSlot, ff);

        this.lastFF = ff;
        this.lastReference = posSetpoint.in(posUnit);

        time = WPIUtilJNI.getSystemTime() - time;
        loopTimeLog.append(time);
    }
}
