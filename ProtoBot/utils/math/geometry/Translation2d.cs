using System;
namespace ProtoBot.utils.math.geometry;

public class Translation2d
{
	private readonly double x;
	private readonly double y;

	public Translation2d()
	{
		x = 0;
		y = 0;
	}

	public Translation2d(double x, double y)
	{
		this.x = x;
		this.y = y;
	}


	public Translation2d(double distance, Rotation2d angle)
	{
		this.x = distance * angle.GetCos();
		this.y = distance * angle.GetSin();
	}

	public double GetY()
	{
		return y;
	}

	public double GetX()
	{
		return x;
	}
}

