package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.prefs.MoPrefs;
import frc.robot.utils.MoUtils;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int ELEVATOR_CURRENT_LIMIT = 50;
    private static final int WRIST_CURRENT_LIMIT = 50;

    private final SparkMax elevatorA;
    private final SparkMax elevatorB;
    private final SparkMax elevatorWrist;
    private final VictorSPX endEffector;

    // public final MoRotationEncoder elevatorAbsEncoder;
    // public final MoRotationEncoder wristAbsEncoder;
    // public final MoRotationEncoder elevatorRelEncoder;
    // public final MoRotationEncoder wristRelEncoder;

    public static record ElevatorPosition(Angle elevatorAngle, Angle wristAngle) {}

    public static record ElevatorMovementRequest(double elevatorPower, double wristPower) {
        public ElevatorMovementRequest(double elevatorPower, double wristPower) {
            this.elevatorPower = MoUtils.clamp(elevatorPower, -1, 1);
            this.wristPower = MoUtils.clamp(wristPower, -1, 1);
        }

        public boolean isZero() {
            return Math.abs(elevatorPower) < Constants.FLOAT_EPSILON && Math.abs(wristPower) < Constants.FLOAT_EPSILON;
        }
    }

    public void configureMotors() {
        this.elevatorA.configure(
                new SparkMaxConfig().smartCurrentLimit(ELEVATOR_CURRENT_LIMIT),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        this.elevatorB.configure(
                new SparkMaxConfig().smartCurrentLimit(ELEVATOR_CURRENT_LIMIT).follow(elevatorA, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        this.elevatorWrist.configure(
                new SparkMaxConfig().smartCurrentLimit(WRIST_CURRENT_LIMIT),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public ElevatorSubsystem() {
        this.elevatorA = new SparkMax(Constants.ELEVATORA.address(), MotorType.kBrushless);
        this.elevatorB = new SparkMax(Constants.ELEVATORB.address(), MotorType.kBrushless);
        this.elevatorWrist = new SparkMax(Constants.ELEVATOR_WRIST.address(), MotorType.kBrushless);
        this.endEffector = new VictorSPX(Constants.END_EFFECTOR.address());
    }

    public void intakeAlgaeCoralExtake() {
        endEffector.set(
                ControlMode.PercentOutput, MoPrefs.endEffectorPower.get().in(Units.Volts));
    }

    public void extakeAlgaeCoralIntake() {
        endEffector.set(
                ControlMode.PercentOutput, -MoPrefs.endEffectorPower.get().in(Units.Volts));
    }
}
