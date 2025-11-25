// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import java.lang.module.ModuleFinder;
import java.util.logging.Logger;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.GyroIO.GyroIOInputs;

public class DrivetrainSubsystem extends SubsystemBase {
  static double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static double DRIVE_BASE_RADIUS = Math.max(
    Math.max(
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
    ),
    Math.max(
      Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    )
  );

  private GyroIO gIO;
  private GyroIOInputs gInputs = new GyroIOInputs();
  private Module[] mods = new Module[4];
  private SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModTrans());
  private Rotation2d rawGRot = new Rotation2d();
  private SwerveModulePosition[] lModPos = new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private SwerveDrivePoseEstimator poseEst = new SwerveDrivePoseEstimator(kinematics, rawGRot, lModPos, new Pose2d());
  private static double ROBOT_MASS = 60.0;
  private static double MOI = 0.04;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem(GyroIO gIO, ModuleIO fLIO, ModuleIO fRIO, ModuleIO bLIO, ModuleIO bRIO) {
    this.gIO = gIO;
    mods[0] = new Module(fLIO, 0, TunerConstants.FrontLeft);
    mods[1] = new Module(fRIO, 1, TunerConstants.FrontRight);
    mods[2] = new Module(bLIO, 2, TunerConstants.BackLeft);
    mods[3] = new Module(bRIO, 3, TunerConstants.BackRight);

    sysId = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null),
      new SysIdRoutine.Mechanism((v) -> runChar(v.in(Volts)), null, this)
    );
  }

  @Override
  public void periodic() {
    gIO.updateInputs(gInputs);
    for (Module m : mods) { m.periodic(); }

    if (DriverStation.isDisabled()) {
      for (Module m : mods) { m.stop(); }
    }

    SwerveModulePosition[] modPos = new SwerveModulePosition[4];
    SwerveModulePosition[] modD = new SwerveModulePosition[4];
    for (int i; i < 4; i++) {
      modPos[i] = mods[i].getPos();
      modD[i] = new SwerveModulePosition(modPos[i].distanceMeters - lModPos[i].distanceMeters, modPos[i].angle);
      lModPos[i] = modPos[i];
    }

    if (gInputs.connect) { rawGRot = gInputs.yPos; }
    else { rawGRot = rawGRot.plus(new Rotation2d(kinematics.toTwist2d(modD).dtheta)); }

    poseEst.update(rawGRot, modPos);
  }

  public void runVel(ChassisSpeeds spds) {
    ChassisSpeeds discSpds = ChassisSpeeds.discretize(spds, 0.02);
    SwerveModuleState[] setStes = kinematics.toSwerveModuleStates(discSpds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setStes, TunerConstants.kSpeedAt12Volts);

    for (int i = 0; i < 4; i++) { mods[i].runSt(setStes[i]); }
  }

  public void runChar(double out) { for (Module m : mods) { m.runChar(out); } }

  public void stop() { runVel(new ChassisSpeeds()); }

  @AutoLogOutput(key = "swervestates")
  private SwerveModuleState[] getModStes() {
    SwerveModuleState[] s = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      s[i] = mods[i].getSte();
    }
    return s;
  }

  private SwerveModulePosition[] getModPos() {
    SwerveModulePosition[] p = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      p[i] = mods[i].getPos();
    }
    return p;
  }
}
