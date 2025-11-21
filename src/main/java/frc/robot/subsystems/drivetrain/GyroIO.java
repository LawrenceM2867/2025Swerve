// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connect = false; //is gyro connected
        public Rotation2d yPos = new Rotation2d(); //yaw position
        public double yVelRadPerS = 0.0; //yaw velocity in radians per second
    }

    public default void updateInputs(GyroIOInputs inputs) {} //updates the inputs
}
