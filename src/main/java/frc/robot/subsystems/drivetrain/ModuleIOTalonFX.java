package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import static frc.robot.util.PhoenixUtil.*;

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
        driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted 
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, .25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0,.25));
        
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource =
            switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            default -> throw new RuntimeException(
                "Unsupported swerve config.");
            };
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration =
            turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted =constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));


        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection =constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);


        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        turnAbsPos = cancoder.getAbsolutePosition();
        turnPos = turnTalon.getPosition();
        turnVel = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Drive.ODOMETRY_FREQUENCY, drivePos, turnPos);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVel,
            driveAppliedVolts,
            driveCurrent,
            turnAbsPos,
            turnVel,
            turnAppliedVolts,
            turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs in){
        var driveStatus = BaseStatusSignal.refreshAll(drivePos,driveVel,driveAppliedVolts,driveCurrent);
        var turnStatus = BaseStatusSignal.refreshAll(turnPos,turnVel,turnAppliedVolts,turnCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsPos);
        in.dConnect = driveConnectedDebounce.calculate(driveStatus.isOK());
        in.dPosRad = Units.rotationsToRadians(drivePos.getValueAsDouble());
        in.dApplyVolts = driveAppliedVolts.getValueAsDouble();
        in.dCurrentAmps = driveAppliedVolts.getValueAsDouble();

        in.tConnect =  turnConnectedDebounce.calculate(turnStatus.isOK());
        in.tEncConnect = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        in.tAbsPos = Rotation2d.fromRotations(turnAbsPos.getValueAsDouble());
        in.tPos = Rotation2d.fromRotations(turnPos.getValueAsDouble());
        in.tVelRadPerS = Units.rotationsToRadians(turnVel.getValueAsDouble());
        in.tApplyVolts = turnAppliedVolts.getValueAsDouble();
        in.tCurrentAmps = turnCurrent.getValueAsDouble();
    }

    @Override
    public void setDOpenLoop(double output){
        driveTalon.setControl(
            switch(constants.SteerMotorClosedLoopOutput){
                case Voltage -> voltageReq.withOutput(output);
                case TorqueCurrentFOC -> torqueCurrentReq.withOutput(output);
            });
    }

    @Override
    public void setTOpenLoop(double output){
        turnTalon.setControl(
            switch(constants.SteerMotorClosedLoopOutput){
                case Voltage -> voltageReq.withOutput(output);
                case TorqueCurrentFOC -> torqueCurrentReq.withOutput(output);
        });
    }

    @Override
    public void setDVel(double velRadPerS){
        double velRotPerSec = Units.radiansToRotations(velRadPerS);
        driveTalon.setControl(
            switch(constants.DriveMotorClosedLoopOutput){
                case Voltage -> velVoltageReq.withVelocity(velRotPerSec);
                case TorqueCurrentFOC -> velTorqueCurrentReq.withVelocity(velRotPerSec);
        });
    }

    @Override
    public void setTPos(Rotation2d rot){
        turnTalon.setControl(
            switch (constants.SteerMotorClosedLoopOutput){
                case Voltage -> posVoltageReq.withPosition(rot.getRotations());
                case TorqueCurrentFOC -> posTorqueCurrentReq.withPosition(rot.getRotations());
        });
    }
}
