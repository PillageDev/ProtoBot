using System;
using ProtoBot.utils.math.geometry;

namespace ProtoBot.utils.math.kinematics;

public class SwerveModulePosition
{
	public double distanceMeters;
	public Rotation2d angle = Rotation2d.FromDegrees(0);

	public SwerveModulePosition(double distanceMeters, Rotation2d angle)
	{
		this.distanceMeters = distanceMeters;
		this.angle = angle;
	}
}

