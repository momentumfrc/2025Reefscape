package frc.robot.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.encoder.MoDistanceEncoder;
import java.util.Optional;
import java.util.function.Consumer;

public class MoSparkMaxElevatorPID extends MoSparkMaxPID<DistanceUnit, LinearVelocityUnit> {
    private Optional<ElevatorFeedforward> elevatorFF = Optional.empty();

    private double kS = 0;
    private double kG = 0;
    private double kV = 0;

    private double lastFF;

    public MoSparkMaxElevatorPID(
            MoSparkMaxPID.Type type,
            SparkBase spark,
            ClosedLoopSlot pidSlot,
            MoDistanceEncoder encoder,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        super(type, spark, pidSlot, encoder, configurator);
    }

    public MoSparkMaxElevatorPID(
            MoSparkMaxPID.Type type, SparkBase spark, ClosedLoopSlot pidSlot, MoDistanceEncoder encoder) {
        this(type, spark, pidSlot, encoder, MoSparkConfigurator.forSparkBase(spark));
    }

    public void setKS(double kS) {
        this.kS = kS;
        this.elevatorFF = Optional.empty();
    }

    public void setKG(double kG) {
        this.kG = kG;
        this.elevatorFF = Optional.empty();
    }

    public void setKV(double kV) {
        this.kV = kV;
        this.elevatorFF = Optional.empty();
    }

    public double getLastFF() {
        return lastFF;
    }

    private double getFF(Measure<LinearVelocityUnit> velocity) {
        if (elevatorFF.isEmpty()) {
            this.elevatorFF = Optional.of(new ElevatorFeedforward(kS, kG, kV));
        }

        return this.elevatorFF.get().calculate(velocity.in(Units.MetersPerSecond));
    }

    @Override
    public void setPositionReference(Measure<DistanceUnit> desiredPosition) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    "Cannot set position reference on PID controller of type " + this.type.name());
        }

        // The velocity component of the FF calcuation is handled by the kFF set on the motor
        // controller, and the arbitrary FF we're adding here is only to linearize the motor response against gravity.
        double ff = getFF(Units.MetersPerSecond.zero());
        double value = desiredPosition.in(internalEncoder.getInternalEncoderUnits());

        this.pidController.setReference(
                value, this.type.innerType, pidSlot, ff, SparkClosedLoopController.ArbFFUnits.kVoltage);
        this.lastFF = ff;
        this.lastSetpoint = value;
    }

    @Override
    public void setVelocityReference(Measure<LinearVelocityUnit> desiredVelocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    "Cannot set velocity reference on PID controller of type " + this.type.name());
        }

        // The velocity component of the FF calcuation is handled by the kFF set on the motor
        // controller, and the arbitrary FF we're adding here is only to linearize the motor response against gravity.
        double ff = getFF(Units.MetersPerSecond.zero());
        double value = desiredVelocity.in(internalEncoder.getInternalEncoderVelocityUnits());

        this.pidController.setReference(
                value, this.type.innerType, pidSlot, ff, SparkClosedLoopController.ArbFFUnits.kVoltage);
        this.lastFF = ff;
        this.lastSetpoint = value;
    }
}
