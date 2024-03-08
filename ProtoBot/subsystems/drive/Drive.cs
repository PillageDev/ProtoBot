using System;
using ProtoBot.utils;

namespace ProtoBot.subsystems.drive
{
	public class Drive: ISubsystemBase
	{
		private static readonly double MAX_LINEAR_SPEED = Units.FeetToMeters(14.5); //TODO: Find this value for sure
		private static readonly double TRACK_WIDTH_X = Units.InchesToMeters(21.73); //TODO: Find this value for sure
		private static readonly double TRACK_WIDTH_Y = Units.InchesToMeters(21.73); //TODO: Find this value for sure
		private static readonly double DRIVE_BASE_RADIUS = MathUtils.Hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
		private static readonly double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

		public static readonly object odometryLock = new();
		// Gyro IO
		// Gyro Inputs
		// Module array

		// Swerve Kinematics
		// Pose2d
		// Rotation2d

		public Drive(/* Gyro and 4 Modules in constructor */)
		{
			// Init gyro stuff
		}

		public void Periodic()
		{

		}

	}
}

