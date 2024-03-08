using System;
using ProtoBot.utils;
using ProtoBot.utils.math.geometry;
using ProtoBot.utils.math.kinematics;

namespace ProtoBot.subsystems.drive;

public class Module
{
    private static readonly double WHEEL_RADIUS = Units.InchesToMeters(2.0); //TODO: Change this
    private static readonly double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;
    public static readonly double ODOMETRY_FREQUENCY = 250.0;

    private readonly IModuleIO io;
    private readonly IModuleIO.ModuleIOInputs inputs = new();
    private readonly int index;

    private Rotation2d? angleSetpoint = null;
    private double? speedSetpoint = null;
    private Rotation2d? turnRelativeOffset = null;
    private double lastPositionMeters = 0.0;
    private SwerveModulePosition[] positionDeltas = Array.Empty<SwerveModulePosition>();

    public Module(IModuleIO io, int index)
    {
        this.index = index;
        this.io = io;

        SetBrakeMode(true);
    }

    public void UpdateInputs()
    {
        io.UpdateInputs(inputs);
    }

    public void Periodic()
    {
        if (angleSetpoint != null)
        {
            io.SetTurnPosition(angleSetpoint);

            if (speedSetpoint != null)
            {
                double angleError = Math.Abs(RotationalDifferenceBetween(inputs.turnPosition, angleSetpoint));
                double adjustSpeedSetpoint = (double) (speedSetpoint * Math.Cos(Units.DegreesToRadians(angleError)));

                double velocityRotationsPerSec = adjustSpeedSetpoint / WHEEL_CIRCUMFERENCE;

                io.SetDriveVelocityRPS(velocityRotationsPerSec);
            }
        }

        positionDeltas = new SwerveModulePosition[1];
        for (int i = 0; i < 1; i++)
        {
            double positionMeters = inputs.drivePositionRotations * WHEEL_CIRCUMFERENCE;
            Rotation2d angle = inputs.turnPosition;
            positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
            lastPositionMeters = positionMeters;
        }
    }

    public SwerveModuleState RunSetpoint(SwerveModuleState state)
    {
        var optimizedState = SwerveModuleState.Optimize(state, GetAngle());

        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    public void Stop()
    {
        io.SetTurnVoltage(0.0);
        io.SetDriveVoltage(0.0);

        angleSetpoint = null;
        speedSetpoint = null;
    }

    public void SetBrakeMode(bool enable)
    {
        io.SetDriveBrakeMode(enable);
        io.SetTurnBrakeMode(enable);
    }

    public void SetDriveBrakeMode(bool enable)
    {
        io.SetDriveBrakeMode(enable);
    }

    public Rotation2d GetAngle()
    {
        return inputs.turnPosition;
    }

    public double GetPositionMeters()
    {
        return inputs.drivePositionRotations * WHEEL_CIRCUMFERENCE;
    }

    public double GetVelocityMetersPerSec()
    {
        return inputs.driveVelocityRotationsPerSec * WHEEL_CIRCUMFERENCE;
    }

    public SwerveModulePosition GetPosition()
    {
        return new SwerveModulePosition(GetPositionMeters(), GetAngle());
    }

    public SwerveModuleState GetState()
    {
        return new SwerveModuleState(GetVelocityMetersPerSec(), GetAngle());
    }

    public SwerveModulePosition[] GetPositionDeltas()
    {
        return positionDeltas;
    }

    public double RotationalDifferenceBetween(Rotation2d a, Rotation2d b)
    {
        double aDeg = a.GetDegrees();
        double bDeg = b.GetDegrees();

        double diff = Math.Abs(aDeg - bDeg) % 360;
        if (diff > 180)
        {
            diff = 360.0 - diff;
        }

        return diff;
    }
}