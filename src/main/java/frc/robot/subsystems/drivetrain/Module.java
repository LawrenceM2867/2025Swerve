// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;

/** Add your docs here. */
public class Module {
    private ModuleIO io;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private int i;
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> cnst;

    public Module(ModuleIO io, int i, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> cnst) {
        this.io = io;
        this.i = i;
        this.cnst = cnst;
    }

    public void periodic() {
        io.updateInputs(inputs);
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

    public double getWheelRadiusCharPos() {
        return inputs.dPosRad;
    }

    public double getFFCharVel() {
        return Units.radiansToRotations(inputs.dVelRadPerS);
    }
}