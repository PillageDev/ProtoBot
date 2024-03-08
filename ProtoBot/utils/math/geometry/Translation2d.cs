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

	public Translation2d Plus(Translation2d other)
	{
		return new Translation2d(x + other.x, y + other.y);
	}

	public Translation2d Minus(Translation2d other)
	{
		return new Translation2d(x - other.x, y - other.y);
	}

	public Translation2d RotateBy(Rotation2d other)
	{
		return new Translation2d(
			x * other.GetCos() - y * other.GetSin(), x * other.GetSin() + y * other.GetCos());
	}

	public Translation2d Times(double scalar)
	{
		return new Translation2d(x * scalar, y * scalar);
	}

	public Rotation2d GetAngle()
	{
		return new Rotation2d(x, y);
	}
}

