using CTRE.Phoenix6;
using CTRE.Phoenix6.Configs;
using CTRE.Phoenix6.Controls;
using CTRE.Phoenix6.Hardware.Core;
using CTRE.Phoenix6.Signals;
using ProtoBot.utils;
using ProtoBot.utils.geometry;

namespace ProtoBot.subsystems.drive;

public class ModuleIOKraken: IModuleIO
{
    private readonly CoreTalonFX driveTalon;
    private readonly CoreTalonFX turnTalon;
    private readonly CoreCANcoder canCoder;

    private readonly StatusSignal<Double> drivePosition;
    private readonly Queue<Double> drivePositionQueue; // Probably not used
    private readonly StatusSignal<Double> driveVelocity;
    private readonly StatusSignal<Double> driveAppliedVolts;
    private readonly StatusSignal<Double> driveCurrent;

    private readonly StatusSignal<Double> turnAbsolutePosition;
    private readonly StatusSignal<Double> turnPosition;
    private readonly Queue<Double> turnPositionQueue; // Probably not used
    private readonly StatusSignal<Double> turnVelocity;
    private readonly StatusSignal<Double> turnAppliedVolts;
    private readonly StatusSignal<Double> turnCurrent;

    private static readonly double DRIVE_GEAR_RATIO = Constants.SwerveConstants.driveGearRatio;
    private static readonly double TURN_GEAR_RATIO = Constants.SwerveConstants.angleGearRatio;

    private static readonly double WHEEL_RADIUS = Units.InchesToMeters(2.0); //TODO: Get this value for sure
    private static readonly double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;
    private static readonly string CANBUS_ID = "Canivore";

    private static VoltageOut voltageOutCommand = new(0.0, true, false);
    private static VelocityVoltage velocityVoltageCommand = new(0.0, 0.0, true, 0.0, 0, false);
    private static PositionVoltage positionVoltageCommand = new(0.0, 0.0, true, 0.0, 0, false);

    private double absoluteEncoderOffset = 0.0;

    public ModuleIOKraken(int index)
    {
        switch (index)
        {
            case 0: //FL
                driveTalon = new CoreTalonFX(0, CANBUS_ID);
                turnTalon = new CoreTalonFX(1, CANBUS_ID);
                canCoder = new CoreCANcoder(2, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: Find this value
                break;
            case 1:
                driveTalon = new CoreTalonFX(3, CANBUS_ID);
                turnTalon = new CoreTalonFX(4, CANBUS_ID);
                canCoder = new CoreCANcoder(5, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: Find this value
                break;
            case 2:
                driveTalon = new CoreTalonFX(6, CANBUS_ID);
                turnTalon = new CoreTalonFX(7, CANBUS_ID);
                canCoder = new CoreCANcoder(8, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: Find this value
                break;
            case 3:
                driveTalon = new CoreTalonFX(9, CANBUS_ID);
                turnTalon = new CoreTalonFX(10, CANBUS_ID);
                canCoder = new CoreCANcoder(11, CANBUS_ID);
                absoluteEncoderOffset = 0.0; //TODO: Find this value
                break;
            default:
                throw new();
        }

        driveTalon.Configurator.Apply(GetDriveConfig());
        //TODO: Clear sticky faults
        SetDriveBrakeMode(true);

        turnTalon.Configurator.Apply(GetTurnConfig());
        SetTurnBrakeMode(true);

        canCoder.Configurator.Apply(GetCanCoderConfig());

        drivePosition = driveTalon.GetPosition();
        //drivePositionQueue = (phoenix odometry thread)
        driveVelocity = driveTalon.GetVelocity();
        driveAppliedVolts = driveTalon.GetSupplyVoltage(); //TODO: Check if this is correct
        driveCurrent = driveTalon.GetStatorCurrent();

        turnAbsolutePosition = canCoder.GetAbsolutePosition();
        //TODO: Set position of turn motor to turnAbsolutePosition (Probably have to change contorl type)
        turnPosition = turnTalon.GetPosition();
        //turnPositionQueue = (phoenix odometry thread)
        turnVelocity = turnTalon.GetVelocity();
        turnAppliedVolts = turnTalon.GetSupplyVoltage(); //TODO: Check if this is correct
        turnCurrent = driveTalon.GetStatorCurrent();

        BaseStatusSignal.SetUpdateFrequencyForAll(
            50.0,
            0.05,
            new BaseStatusSignal[] { drivePosition, turnPosition, driveVelocity, turnVelocity, driveAppliedVolts, turnAppliedVolts, driveCurrent, turnCurrent });
    }

    public void UpdateInputs(IModuleIO.ModuleIOInputs inputs)
    {
        BaseStatusSignal.RefreshAll(drivePosition, turnPosition, driveVelocity, turnVelocity, driveAppliedVolts, turnAppliedVolts, driveCurrent, turnCurrent);

        inputs.canCoderRotations = canCoder.GetAbsolutePosition().ValueAsDouble;
        inputs.canCoderAngle = Units.RotationsToDegrees(inputs.canCoderRotations);

        inputs.drivePositionRotations = drivePosition.ValueAsDouble;
        inputs.driveVelocityRotationsPerSec = driveVelocity.ValueAsDouble;
        inputs.driveAppliedVolts = driveAppliedVolts.ValueAsDouble;
        inputs.driveCurrentAmps = new double[] { driveCurrent.ValueAsDouble };

        inputs.turnAbsolutePosition = Rotation2d.FromRotations(turnAbsolutePosition.ValueAsDouble);
        inputs.turnPosition = Rotation2d.FromRotations(turnPosition.ValueAsDouble);
        inputs.turnPositionAngle = inputs.turnPosition.GetDegrees();
        inputs.turnVelocityRadPerSec = Units.DegreesToRadians(turnVelocity.ValueAsDouble);
        inputs.turnAppliedVolts = turnAppliedVolts.ValueAsDouble;
        inputs.turnCurrentAmps = new double[] { turnCurrent.ValueAsDouble };
    }

    private TalonFXConfiguration GetDriveConfig()
    {
        TalonFXConfiguration config = new();
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;

        config.Slot0.kP = Constants.SwerveConstants.driveKD;
        config.Slot0.kI = Constants.SwerveConstants.driveKI;
        config.Slot0.kD = Constants.SwerveConstants.driveKD;

        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        return config;
    }

    private TalonFXConfiguration GetTurnConfig()
    {
        TalonFXConfiguration config = new();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.Inverted = Constants.SwerveConstants.angleMotorInvert;

        config.Slot0.kP = Constants.SwerveConstants.angleKP;
        config.Slot0.kI = Constants.SwerveConstants.angleKI;
        config.Slot0.kD = Constants.SwerveConstants.angleKD;

        config.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;

        return config;
    }

    private CANcoderConfiguration GetCanCoderConfig()
    {
        CANcoderConfiguration config = new();
        config.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderSensorDirection;
        config.MagnetSensor.MagnetOffset = -absoluteEncoderOffset; //TODO: make sure this is true

        return config;
    }

    public void SetDriveVoltage(double volts)
    {
        voltageOutCommand.Output = volts;
        driveTalon.SetControl(voltageOutCommand);
    }

    public void SetDriveVelocityRPS(double velocity)
    {
        velocityVoltageCommand.Velocity = velocity;
        velocityVoltageCommand.Slot = 0;
        driveTalon.SetControl(velocityVoltageCommand);
    }

    public void SetTurnVoltage(double volts)
    {
        voltageOutCommand.Output = volts;
        turnTalon.SetControl(voltageOutCommand);
    }

    public void SetTurnPosition(Rotation2d position)
    {
        positionVoltageCommand.Position = position.GetRotations();
        positionVoltageCommand.Slot = 0;
        turnTalon.SetControl(positionVoltageCommand);
    }

    public void SetDriveBrakeMode(bool enable)
    {
        TalonFXConfiguration config = new();
        config.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;
        config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        driveTalon.Configurator.Apply(config);
    }

    public void SetTurnBrakeMode(bool enable)
    {
        TalonFXConfiguration config = new();
        config.MotorOutput.Inverted = Constants.SwerveConstants.angleMotorInvert;
        config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        turnTalon.Configurator.Apply(config);
    }
}