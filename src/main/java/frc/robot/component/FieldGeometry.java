package frc.robot.component;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.EnumMap;

public class FieldGeometry {
    private static final Distance BRANCH_DISTANCE_FROM_FACE_CENTER = Units.Inches.of(12.94 / 2);

    public enum ReefFace {
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL;
    }

    public enum ReefBranch {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L;
    }

    public enum RelativeBranch {
        LEFT,
        RIGHT;
    }

    private static FieldGeometry instance;

    public static synchronized FieldGeometry getInstance() {
        if (instance == null) {
            instance = new FieldGeometry();
        }
        return instance;
    }

    private EnumMap<ReefFace, Pose2d> facePosesBlue = new EnumMap<>(ReefFace.class);
    private EnumMap<ReefFace, Pose2d> facePosesRed = new EnumMap<>(ReefFace.class);

    public Pose2d getFacePose(ReefFace face, Alliance alliance) {
        return switch (alliance) {
            case Blue -> facePosesBlue.get(face);
            case Red -> facePosesRed.get(face);
        };
    }

    public ReefFace faceForBranch(ReefBranch branch) {
        return switch (branch) {
            case A -> ReefFace.AB;
            case B -> ReefFace.AB;
            case C -> ReefFace.CD;
            case D -> ReefFace.CD;
            case E -> ReefFace.EF;
            case F -> ReefFace.EF;
            case G -> ReefFace.GH;
            case H -> ReefFace.GH;
            case I -> ReefFace.IJ;
            case J -> ReefFace.IJ;
            case K -> ReefFace.KL;
            case L -> ReefFace.KL;
        };
    }

    public RelativeBranch relativeForAbsolute(ReefBranch branch) {
        return switch (branch) {
            case A -> RelativeBranch.LEFT;
            case B -> RelativeBranch.RIGHT;
            case C -> RelativeBranch.LEFT;
            case D -> RelativeBranch.RIGHT;
            case E -> RelativeBranch.LEFT;
            case F -> RelativeBranch.RIGHT;
            case G -> RelativeBranch.LEFT;
            case H -> RelativeBranch.RIGHT;
            case I -> RelativeBranch.LEFT;
            case J -> RelativeBranch.RIGHT;
            case K -> RelativeBranch.LEFT;
            case L -> RelativeBranch.RIGHT;
        };
    }

    public Pose2d getBranchPose(ReefBranch branch, Alliance alliance) {
        ReefFace face = faceForBranch(branch);
        RelativeBranch relative = relativeForAbsolute(branch);

        double offset = BRANCH_DISTANCE_FROM_FACE_CENTER.in(Units.Meters);
        if (relative == RelativeBranch.RIGHT) {
            offset *= -1;
        }

        Transform2d transform = new Transform2d(offset, 0, Rotation2d.kZero);
        return getFacePose(face, alliance).transformBy(transform);
    }

    private FieldGeometry() {
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        int[] reefFaceApriltagsBlue = {18, 17, 22, 21, 20, 19};
        int[] reefFaceApriltagsRed = {10, 11, 6, 7, 8, 9};

        for (int i = 0; i < ReefFace.values().length; i++) {
            ReefFace curr = ReefFace.values()[i];
            facePosesBlue.put(
                    curr, fieldLayout.getTagPose(reefFaceApriltagsBlue[i]).get().toPose2d());
            facePosesRed.put(
                    curr, fieldLayout.getTagPose(reefFaceApriltagsRed[i]).get().toPose2d());
        }
    }
}
