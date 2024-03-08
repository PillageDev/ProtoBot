using System;
using ProtoBot.utils.math.geometry;

namespace ProtoBot.utils.math.kinematics;

public class SwerveModuleState
{
	public double speedMetersPerSecond;
	public Rotation2d angle = Rotation2d.FromDegrees(0.0);

	public SwerveModuleState()
	{
	}

	public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle)
	{
		this.speedMetersPerSecond = speedMetersPerSecond;
		this.angle = angle;
	}

	public static SwerveModuleState Optimize(SwerveModuleState desiredState, Rotation2d currentAngle)
	{
		var delta = desiredState.angle.Minus(currentAngle);
		if (Math.Abs(delta.GetDegrees()) > 90)
		{
			return new SwerveModuleState(
				-desiredState.speedMetersPerSecond,
				desiredState.angle.RotateBy(Rotation2d.FromDegrees(180.0)));
		} else
		{
			return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
		}
	}
}

