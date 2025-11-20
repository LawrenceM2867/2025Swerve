// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class GyroIOPigeon2 implements GyroIO {
    private Pigeon2 g = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id);
    private StatusSignal<Angle> y = g.getYaw();
    private StatusSignal<AngularVelocity> yVel = g.getAngularVelocityZWorld();

    public GyroIOPigeon2() {
        g.getConfigurator().apply(new Pigeon2Configuration());
        g.getConfigurator().setYaw(0.0);
        g.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

    }
}
