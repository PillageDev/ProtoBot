using System;
using ProtoBot.utils;
using ProtoBot.utils.math.geometry;
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

    private SwerveDriveKinematics kinematics = new(GetModuleTranslations());
    private Pose2d pose = new();
    private Rotation2d lastGyroRotation = new();

    public Drive(IGyroIO gyroIO, IModuleIO fl, IModuleIO fr, IModuleIO bl, IModuleIO br)
    {
        this.gyroIO = gyroIO;
        modules[0] = new Module(fl, 0);
        modules[1] = new Module(fr, 1);
        modules[2] = new Module(bl, 2);
        modules[3] = new Module(br, 3);
    }

    public override void Periodic()
    {
        gyroIO.UpdateInputs(gyroInputs);
        foreach (var module in modules)
        {
            module.UpdateInputs();
        }

        foreach (var module in modules)
        {
            module.Periodic();
        }

        // Check if the robot is disabled, and if so stop the modules

        int deltaCount = int.MaxValue;

        for (int i = 0; i < 4; i++)
        {
            deltaCount = Math.Min(deltaCount, modules[i].GetPositionDeltas().Length);
        }

        for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++)
        {
            SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
            {
                wheelDeltas[moduleIndex] = modules[moduleIndex].GetPositionDeltas()[deltaIndex];
            }

            var twist = kinematics.toTwist2d(wheelDeltas);
        }
    }

    public override bool End()
    {
        return false;
    }

    public static Translation2d[] GetModuleTranslations()
    {
        return new Translation2d[]
        {
            new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
            new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
            new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
            new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
        };
    }
}