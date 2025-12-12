// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class GyroIOPigeon2 implements GyroIO {
    private Pigeon2 g = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id); //creates a new gyro
    private StatusSignal<Angle> y = g.getYaw(); //value for the yaw
    private StatusSignal<AngularVelocity> yVel = g.getAngularVelocityZWorld(); //value for the yaw velocity

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon2() {
        g.getConfigurator().apply(new Pigeon2Configuration());
        g.getConfigurator().setYaw(0.0);
        g.optimizeBusUtilization();

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(y.clone());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connect = BaseStatusSignal.refreshAll(y, yVel).equals(StatusCode.OK);
        inputs.yPos = Rotation2d.fromDegrees(y.getValueAsDouble());
        inputs.yVelRadPerS = Units.degreesToRadians(yVel.getValueAsDouble());
        
        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
            yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}