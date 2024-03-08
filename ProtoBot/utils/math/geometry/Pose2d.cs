using System;
namespace ProtoBot.utils.math.geometry
{
	public class Pose2d
	{
		private readonly Translation2d translation;
		private readonly Rotation2d rotation;

		public Pose2d()
		{
			translation = new();
			rotation = new();
		}

		public Pose2d(Translation2d translation, Rotation2d rotation)
		{
			this.translation = translation;
			this.rotation = rotation;
		}

		public Pose2d(double x, double y, Rotation2d rotation)
		{
			this.translation = new Translation2d(x, y);
			this.rotation = rotation;
		}
	}
}

