using System;
using ProtoBot.utils;
using ProtoBot.utils.geometry;

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
    // Swerve module position

    public Module(IModuleIO io, int index)
    {
        this.index = index;
        this.io = io;

        setBrakeMode(true);
    }

    public void updateInputs()
    {
        io.updateInputs(inputs);
    }

    public void periodic()
    {
        if (angleSetpoint != null)
        {
            io.SetTurnPosition(angleSetpoint);

            if (speedSetpoint != null)
            {
                double angleError = Math.Abs(rotationalDifferenceBetween(inputs.turnPosition, angleSetpoint));
                double adjustSpeedSetpoint = speedSetpoint * Math.Cos(Units.DegreesToRadians(angleError));

                double velocityRotationsPerSec = adjustSpeedSetpoint / WHEEL_CIRCUMFERENCE;

                io.SetDriveVelocityRPS(velocityRotationsPerSec);
            }
        }
    }

    public double rotationalDifferenceBetween(Rotation2d a, Rotation2d b)
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