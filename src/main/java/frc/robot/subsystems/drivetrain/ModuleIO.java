// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean dConnect = false;
        public double dPosRad = 0.0;
        public double dVelRadPerS = 0.0;
        public double dApplyVolts = 0.0;
        public double dCurrentAmps = 0.0;
        public double dTempC = 0.0;

        public boolean tConnect = false;
        public boolean tEncConnect = false; 
        public Rotation2d tAbsPos = new Rotation2d(); 
        public Rotation2d tPos = new Rotation2d(); 
        public double tVelRadPerS = 0.0; 
        public double tApplyVolts = 0.0; 
        public double tCurrentAmps = 0.0; 
        public double tTempC = 0.0; 
        public double encOffset = 0.0;
    
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}
    public default void setDOpenLoop(double output) {} 
    public default void setTOpenLoop(double output) {}
    public default void setDVel(double velRadPerS) {}
    public default void setTPos(Rotation2d rot) {}
}
