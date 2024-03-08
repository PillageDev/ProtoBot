using System;
using ProtoBot.utils;
using ProtoBot.utils.math.kinematics;

namespace ProtoBot.subsystems.drive;

public class Drive: SubsystemBase
{
    private static readonly double MAX_LINEAR_SPEED = Units.FeetToMeters(14.5); //TODO: Find this value for sure
    private static readonly double TRACK_WIDTH_X = Units.InchesToMeters(21.73); //TODO: Find this value for sure
    private static readonly double TRACK_WIDTH_Y = Units.InchesToMeters(21.73); //TODO: Find this value for sure
    private static readonly double DRIVE_BASE_RADIUS = MathUtils.Hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    private static readonly double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static readonly object OdometryLock = new();
    private readonly IGyroIO gyroIO;
    private readonly IGyroIO.GyroIOInputs gyroInputs = new();
    private readonly Module[] modules = new Module[4]; // FL, FR, BL, BR

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics();
    // Pose2d
    // Rotation2d

    public Drive(IGyroIO gyro, IModuleIO fl, IModuleIO fr, IModuleIO bl, IModuleIO br)
    {
        // Init gyro stuff
    }

    public override void Periodic()
    {

    }

    public override bool End()
    {
        return false;
    }

}