// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Module {
    private ModuleIO io;
    private final int index;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> cnst;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> cnst) {
        this.io = io;
        this.cnst = cnst;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * cnst.WheelRadius;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

    }

    public void runSt(SwerveModuleState ste) {
        ste.optimize(getAngle());
        ste.cosineScale(inputs.tPos);

        io.setDVel(ste.speedMetersPerSecond / cnst.WheelRadius);
        io.setTPos(ste.angle);
    }

    public void runChar(double out) {
        io.setDOpenLoop(out);
        io.setTPos(new Rotation2d());
    }

    public void stop() {
        io.setDOpenLoop(0.0);
        io.setTOpenLoop(0.0);
    }

    public Rotation2d getAngle() {
        return inputs.tPos;
    }

    public double getPosM() {
        return inputs.dPosRad * cnst.WheelRadius;
    }

    public double getVelMPerS() {
        return inputs.dVelRadPerS * cnst.WheelRadius;
    }

    public SwerveModulePosition getPos() {
        return new SwerveModulePosition(getPosM(), getAngle());
    }

    public SwerveModuleState getSte() {
        return new SwerveModuleState(getVelMPerS(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getWheelRadiusCharPos() {
        return inputs.dPosRad;
    }

    public double getFFCharVel() {
        return Units.radiansToRotations(inputs.dVelRadPerS);
    }
}