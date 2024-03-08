using System;
using ProtoBot.utils.geometry;

namespace ProtoBot.subsystems.drive
{
	public interface IGyroIO
	{
		public class GyroIOInputs
		{
			public bool connected = false;
			public Rotation2d realYawPosition = new();
			public Rotation2d yawPosition = new();
			public Rotation2d yawOffset = new();
			public Rotation2d[] odometryYawPosition = Array.Empty<Rotation2d>();
			public Rotation3d pose;
			public double yawVelocityRadPerSec = 0.0;
		}

		public void UpdateInputs(GyroIOInputs inputs);
	}
}

