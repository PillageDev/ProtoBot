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

            var twist = kinematics.ToTwist2d(wheelDeltas);
            if (gyroInputs.connected)
            {
                Rotation2d gyroRotation = gyroInputs.yawPosition;
                twist = new Twist2d(twist.dx, twist.dy, gyroRotation.Minus(lastGyroRotation).GetRadians() / deltaCount);
            }
            pose = pose.Exp(twist);
        }
    }

    public void ResetRotation()
    {
        gyroInputs.yawOffset = gyroInputs.realYawPosition;
        var currentPose = GetPose();
        SetPose(new Pose2d(currentPose.GetTranslation(), new Rotation2d())); // preserve position but reset rotation
    }

    public void RunVelocity(ChassisSpeeds speeds)
    {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.Discretize(speeds, 0.02); //0.02 is the can bus frequency, update for canivore
        SwerveModuleState[] setpointStates = kinematics.ToSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.DesaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
        {
            optimizedStates[i] = modules[i].RunSetpoint(setpointStates[i]);
        }
    }

    public void RunFrontWheelDrive(double speedMetersPerSecond, double angle, bool brake, double maxTurningAngle)
    {
        SwerveModuleState frontWheels = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(0).Minus(Rotation2d.FromDegrees(angle * maxTurningAngle)));
        SwerveModuleState backWheels = new SwerveModuleState(speedMetersPerSecond, new Rotation2d(0));
        SwerveModuleState[] setpointStates =
        {
            SwerveModuleState.Optimize(frontWheels, new Rotation2d()),
            SwerveModuleState.Optimize(frontWheels, new Rotation2d()),
            SwerveModuleState.Optimize(backWheels, new Rotation2d()),
            SwerveModuleState.Optimize(backWheels, new Rotation2d())
        };
        SwerveDriveKinematics.DesaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
        {
            optimizedStates[i] = modules[i].RunSetpoint(setpointStates[i]);

            if (brake)
            {
                modules[i].SetDriveBrakeMode(true);
                continue;
            }

            modules[i].SetDriveBrakeMode(false);
        }
    }

    public void Stop()
    {
        RunVelocity(new ChassisSpeeds());
    }

    public ChassisSpeeds GetChassisSpeeds()
    {
        return kinematics.ToChassisSpeeds(modules[0].GetState(), modules[1].GetState(), modules[2].GetState(), modules[3].GetState());
    }

    public void StopWithX()
    {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++)
        {
            headings[i] = GetModuleTranslations()[i].GetAngle();
        }
        kinematics.ResetHeadings(headings);
        Stop();
    }

    // This is an autolog, please log
    public SwerveModuleState[] GetModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
        {
            states[i] = modules[i].GetState();
        }
        return states;
    }

    public override bool End()
    {
        return false;
    }

    // This is an autolog, please log
    public Pose2d GetPose()
    {
        return pose;
    }

    public void SetPose(Pose2d pose)
    {
        this.pose = pose;
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