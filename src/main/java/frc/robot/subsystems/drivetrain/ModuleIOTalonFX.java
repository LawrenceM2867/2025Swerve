package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ModuleIOTalonFX implements ModuleIO {
    // First is drive talon, second turn talon
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    private final VoltageOut voltageReq = new VoltageOut(0);
    private final PositionVoltage posVoltageReq = new PositionVoltage(0.0);
    private final VelocityVoltage velVoltageReq = new VelocityVoltage(0.0);

    private final TorqueCurrentFOC torqueCurrentReq = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC posTorqueCurrentReq = new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velTorqueCurrentReq = new VelocityTorqueCurrentFOC(0.0);

    private final StatusSignal<Angle> drivePos;
    private final StatusSignal<AngularVelocity> driveVel;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    private final StatusSignal<Angle> turnAbsPos;
    private final StatusSignal<Angle> turnPos;
    private final StatusSignal<AngularVelocity> turnVel;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    private final Debouncer driveConnectedDebounce = new Debouncer(.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFX(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> c){
        this.constants = c;
        driveTalon = new TalonFX(constants.DriveMotorId, Tuner)

    }
}
