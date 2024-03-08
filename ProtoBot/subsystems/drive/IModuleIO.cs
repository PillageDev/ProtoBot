using System;
using ProtoBot.utils.geometry;

namespace ProtoBot.subsystems.drive;

public interface IModuleIO
{
    public class ModuleIOInputs
    {
        public double drivePositionRotations = 0.0;
        public double driveVelocityRotationsPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] { };

        public Rotation2d turnAbsolutePosition = new();
        public Rotation2d turnPosition = new();
        public double turnPositionAngle = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] { };

        public double canCoderRotations = 0.0;
        public double canCoderAngle = 0.0;
    }

    public void UpdateInputs(ModuleIOInputs inputs);

    public void SetDriveVoltage(double volts);

    public void SetDriveVelocityRPS(double velocity);

    public void SetTurnVoltage(double volts);

    public void SetTurnPosition(Rotation2d position);

    public void SetDriveBrakeMode(bool enable);

    public void SetTurnBrakeMode(bool enable);
}