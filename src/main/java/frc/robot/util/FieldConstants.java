// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Field Constants for the First Robotics Competition 2025 Game Reefscape
 *
 * <pre>
 * _  _________________________  _
 *   /         | | | |         \
 *  |     /\   | | | |   /\     |
 *  |    |  |  | |=| |  |  |    |
 *  |     \/   | | | |   \/     |
 * _ \_________|_|_|_|_________/ _
 * </pre>
 */
public class FieldConstants {
  public static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = fieldLayout.getFieldLength();
  public static final double fieldWidth = fieldLayout.getFieldWidth();
  public static final double widthBetweenPegs =
      0.328619; // Width Between Peg in meters ALWAYS go and check the field
  // BEFORE COMPETITION
  public static final double safeDistance = 0.15;
  public static final LoggedNetworkNumber algaeXOffset =
      new LoggedNetworkNumber("/Tuning/AlgaeXOffset", 18);
  public static final LoggedNetworkNumber loggedYOffset =
      new LoggedNetworkNumber("/Tuning/AlgaeYOffset", 20.0);
  public static final double algaeOffset = Units.inchesToMeters(18.0);
  public static final double algaeYOffset = -Units.inchesToMeters(20.0);
  public static final double algaeL4 = -Units.inchesToMeters(18);

  public static class ReefConstants {
    public enum AlgaeTarget {
      L2(0.42),
      L3(0.81);

      public double height;

      private AlgaeTarget(double height) {
        this.height = height;
      }
    }

    public static Pose2d[] aprilTags =
        new Pose2d[] {
          fieldLayout.getTagPose(17).get().toPose2d(),
          fieldLayout.getTagPose(18).get().toPose2d(),
          fieldLayout.getTagPose(19).get().toPose2d(),
          fieldLayout.getTagPose(20).get().toPose2d(),
          fieldLayout.getTagPose(21).get().toPose2d(),
          fieldLayout.getTagPose(22).get().toPose2d()
        };
    public static Pose2d[] leftBranches =
        new Pose2d[] {
          aprilTags[0].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[1].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[2].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[3].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg))
        };
    public static Pose2d[] rightBranches =
        new Pose2d[] {
          aprilTags[0].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[1].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[2].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[3].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg))
        };

    public static Pose2d[] algaeLocations =
        new Pose2d[] {
          aprilTags[0].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg)),
          aprilTags[1].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg)),
          aprilTags[2].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg)),
          aprilTags[3].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))
        };

    private static List<Pose2d> leftBranchList = List.of(leftBranches);
    private static List<Pose2d> rightBranchList = List.of(rightBranches);
    public static List<Pose2d> algaePoses = List.of(algaeLocations);
    public static List<Pose2d> reefAlgae = List.of(algaeLocations);

    private static double L4Offset = 0.25;

    public static Pose2d getBestBranch(Supplier<Pose2d> poseSupplier, boolean left, boolean L4) {
      Pose2d pose = getNearestFlipped(poseSupplier, left ? leftBranchList : rightBranchList);
      Logger.recordOutput(
          "AutoAlign/L4TargetPose",
          pose.transformBy(new Transform2d(-L4Offset, 0.0, Rotation2d.kZero)));
      Logger.recordOutput("AutoAlign/Target", pose);

      return L4 ? pose.transformBy(new Transform2d(-L4Offset, 0.0, Rotation2d.kZero)) : pose;
    }

    public static final Pose2d middleReef = new Pose2d(4.47, 4.03, Rotation2d.k180deg);

    public static Pose2d getReef() {
      return AllianceFlipUtil.apply(middleReef);
    }

    public static boolean nearReef(Supplier<Pose2d> poseSupplier) {
      return getReef().getTranslation().getDistance(poseSupplier.get().getTranslation()) < 3.0;
    }
  }

  public class SourceConstants {
    public static Pose2d[] sourceTags =
        new Pose2d[] {
          fieldLayout.getTagPose(12).get().toPose2d(), fieldLayout.getTagPose(13).get().toPose2d(),
        };
    public static Pose2d[] sourcePoses =
        new Pose2d[] {
          sourceTags[0].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero)),
          sourceTags[1].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero))
        };

    private static List<Pose2d> sourceList = List.of(sourcePoses);

    public static Pose2d getNearestSource(Supplier<Pose2d> poseSupplier) {
      return getNearestFlipped(poseSupplier, sourceList);
    }
  }

  public class BargeConstants {
    public static Pose2d[] bargeTags =
        new Pose2d[] {
          fieldLayout.getTagPose(14).get().toPose2d(), fieldLayout.getTagPose(5).get().toPose2d()
        };
    public static Pose2d[] bargePoses =
        new Pose2d[] {
          bargeTags[0].transformBy(new Transform2d(safeDistance * 1.5, 0.0, Rotation2d.k180deg)),
          bargeTags[1].transformBy(new Transform2d(safeDistance * 1.5, 0.0, Rotation2d.k180deg))
        };
    public static final Pose2d[] climbPoses =
        new Pose2d[] {
          bargeTags[0].transformBy(
              new Transform2d(
                  0.25,
                  Units.inchesToMeters(-0.99995 - 0.25) - Units.inchesToMeters(42.937416),
                  Rotation2d.kZero)),
          bargeTags[0].transformBy(
              new Transform2d(0.25, Units.inchesToMeters(-0.99995 - 0.25), Rotation2d.kZero)),
          bargeTags[0].transformBy(
              new Transform2d(
                  0.25,
                  Units.inchesToMeters(-0.99995 - 0.25) + Units.inchesToMeters(42.937500),
                  Rotation2d.kZero))
        };
    public static final double elevatorSetpoint = 1.1;

    public static boolean nearNet(Supplier<Pose2d> poseSupplier) {
      Logger.recordOutput(
          "Superstructure/NearNet", inNetZone(getNearestNet(poseSupplier), poseSupplier.get()));
      return inNetZone(getNearestNet(poseSupplier), poseSupplier.get());
    }

    public static boolean inNetZone(Pose2d netPose, Pose2d currentPose) {
      return MathUtil.isNear(netPose.getX(), currentPose.getX(), 0.80)
          && MathUtil.isNear(netPose.getY(), currentPose.getY(), FieldConstants.fieldWidth / 2);
    }

    private static List<Pose2d> bargePoseList = List.of(bargePoses);

    public static Pose2d getNearestNet(Supplier<Pose2d> poseSupplier) {
      return getNearestFlipped(poseSupplier, bargePoseList);
    }
  }

  public class ProcessorConstants {
    public static final double elevatorSetpoint = 0.0;
  }

  public class DistanceCheckers {

    public static final Pose2d[] processorTags = {
      fieldLayout.getTagPose(3).get().toPose2d(), fieldLayout.getTagPose(16).get().toPose2d()
    };

    public static final Pose2d[] lowAlgaeTags = {
      fieldLayout.getTagPose(17).get().toPose2d(),
      fieldLayout.getTagPose(19).get().toPose2d(),
      fieldLayout.getTagPose(21).get().toPose2d(),
    };

    public static final Pose2d[] highAlgaeTags = {
      fieldLayout.getTagPose(18).get().toPose2d(),
      fieldLayout.getTagPose(20).get().toPose2d(),
      fieldLayout.getTagPose(22).get().toPose2d(),
    };

    public static Pose2d[] lowAlgaePoses =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              lowAlgaeTags[0].transformBy(
                  new Transform2d(
                      new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
          AllianceFlipUtil.apply(
              lowAlgaeTags[1].transformBy(
                  new Transform2d(
                      new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
          AllianceFlipUtil.apply(
              lowAlgaeTags[2].transformBy(
                  new Transform2d(
                      new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg)))
        };

    public static Pose2d[] highAlgaePoses =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              highAlgaeTags[0].transformBy(
                  new Transform2d(
                      new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
          AllianceFlipUtil.apply(
              highAlgaeTags[1].transformBy(
                  new Transform2d(
                      new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
          AllianceFlipUtil.apply(
              highAlgaeTags[2].transformBy(
                  new Transform2d(
                      new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg)))
        };

    public static List<Pose2d> lowAlgae = List.of(lowAlgaePoses);
    public static List<Pose2d> highAlgae = List.of(highAlgaePoses);
    public static List<Pose2d> algaeList =
        Stream.concat(lowAlgae.stream(), highAlgae.stream()).collect(Collectors.toList());

    public static enum IntakeSource {
      L2,
      L3,
      Ground_Algae
    }

    public static enum Target {
      Processor,
      Barge
    }

    public static Pose2d getRotationToFace(Supplier<Pose2d> drive) {
      return drive.get().nearest(algaeList);
    }

    public static final Target getNearestScore(Supplier<Pose2d> currentPose) {
      Pose2d pose = currentPose.get();
      Pose2d processor = AllianceFlipUtil.shouldFlip() ? processorTags[0] : processorTags[1];
      Pose2d barge =
          AllianceFlipUtil.shouldFlip()
              ? BargeConstants.bargePoses[1]
              : BargeConstants.bargePoses[0];
      if (pose.getTranslation().getDistance(processor.getTranslation())
          <= pose.getTranslation().getDistance(barge.getTranslation())) {
        return Target.Processor;
      } else {
        return Target.Barge;
      }
    }

    public static final Pose2d getBestAlgaeIntakePose(
        Supplier<Pose2d> supplier, IntakeSource source) {
      Pose2d drive = supplier.get();
      return drive.nearest(lowAlgae);

      // if (IntakeSource.Ground_Algae == source) {
      //   return drive.nearest(Camera.returnArrayOfSeenAlgae());
      // }

      // if (IntakeSource.L2 == source) {
      //   return drive.nearest(lowAlgae);
      // }

      // if (IntakeSource.L3 == source) {
      //   return drive.nearest(highAlgae);
      // }

    }
  }

  private static Pose2d endPose = new Pose2d(fieldLength, fieldWidth, Rotation2d.kZero);

  public static boolean inTolerance(
      Supplier<Pose2d> pose1,
      Supplier<Pose2d> pose2,
      double translationTolerance,
      double orientationTolerance) {
    return MathUtil.isNear(pose1.get().getX(), pose2.get().getX(), translationTolerance)
        && MathUtil.isNear(pose1.get().getY(), pose2.get().getY(), translationTolerance)
        && MathUtil.isNear(
            pose1.get().getRotation().getRadians(),
            pose2.get().getRotation().getRadians(),
            orientationTolerance);
  }

  public static Pose2d getNearestFlipped(Supplier<Pose2d> poseSupplier, List<Pose2d> poses) {
    return AllianceFlipUtil.apply(AllianceFlipUtil.apply(poseSupplier.get()).nearest(poses));
  }

  public class AutoConstants {
    public static Pose2d[] aprilTags = {
      fieldLayout.getTagPose(5).get().toPose2d(),
      fieldLayout.getTagPose(17).get().toPose2d(),
      fieldLayout.getTagPose(19).get().toPose2d(),
      fieldLayout.getTagPose(21).get().toPose2d(),
      fieldLayout.getTagPose(18).get().toPose2d(),
      fieldLayout.getTagPose(20).get().toPose2d(),
      fieldLayout.getTagPose(22).get().toPose2d(),
    };

    public static Pose2d[] offsetedPose = {
      AllianceFlipUtil.apply(
          aprilTags[0].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
      AllianceFlipUtil.apply(
          aprilTags[1].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
      AllianceFlipUtil.apply(
          aprilTags[2].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
      AllianceFlipUtil.apply(
          aprilTags[0].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
      AllianceFlipUtil.apply(
          aprilTags[1].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
      AllianceFlipUtil.apply(
          aprilTags[2].transformBy(
              new Transform2d(new Translation2d(algaeOffset, algaeYOffset), Rotation2d.k180deg))),
    };
  }

  public static void Log() {
    Logger.recordOutput("Field Constants/End Point", endPose);
    Logger.recordOutput("Field Constants/Reef/AprilTags", ReefConstants.aprilTags);
    Logger.recordOutput("Field Constants/Reef/Left Branches", ReefConstants.leftBranches);
    Logger.recordOutput("Field Constants/Reef/Right Branches", ReefConstants.rightBranches);
    Logger.recordOutput("Field Constants/Reef/Algae Poses", ReefConstants.algaeLocations);
    Logger.recordOutput("Field Constants/Source/Source Tags", SourceConstants.sourceTags);
    Logger.recordOutput("Field Constants/Source/Source Poses", SourceConstants.sourcePoses);
    Logger.recordOutput("Field Constants/Barge/Barge Tags", BargeConstants.bargeTags);
    Logger.recordOutput("Field Constants/Barge/Barge Poses", BargeConstants.bargePoses);
    Logger.recordOutput(
        "Field Constants/Reef/Middle", AllianceFlipUtil.apply(ReefConstants.middleReef));
    Logger.recordOutput("Field Constants/Barge/Cage Poses", BargeConstants.climbPoses);
    Logger.recordOutput("Field Constants/Current Match Time", Timer.getMatchTime());
    Logger.recordOutput(
        "Field Constants/Current Alliance",
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get()
            : DriverStation.Alliance.Blue);
  }
}