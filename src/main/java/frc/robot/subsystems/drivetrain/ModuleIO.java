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
        public boolean dConnect = false; //is drive connected
        public double dPosRad = 0.0; //drive position in radians
        public double dVelRadPerS = 0.0; //drive velocity in radians per second
        public double dApplyVolts = 0.0; //drive applied voltage
        public double dCurrentAmps = 0.0; //drive current in amps
        public double dTempC = 0.0; //drive temperature in celsius

        public boolean tConnect = false; //is turn connected
        public boolean tEncConnect = false; //is turn encoder connected
        public Rotation2d tAbsPos = new Rotation2d(); //turn absolute position
        public Rotation2d tPos = new Rotation2d(); //turn position
        public double tVelRadPerS = 0.0; //turn velocity in radians per second
        public double tApplyVolts = 0.0; //turn applied voltage
        public double tCurrentAmps = 0.0; //turn current in amps
        public double tTempC = 0.0; //turn temperature in celsius
        public double encOffset = 0.0; //encoder offset in radians
    
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {} //updates the inputs
    public default void setDOpenLoop(double output) {} //runs the drive motor output with open loop control
    public default void setTOpenLoop(double output) {} //runs the turn motor output with open loop control
    public default void setDVel(double velRadPerS) {} //runs the drive motor at the velocity
    public default void setTPos(Rotation2d rot) {} //turns the motor to the position
}
