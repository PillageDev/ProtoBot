using System;
namespace ProtoBot.utils.math.geometry;

public class Rotation2d
{
	private readonly double value;
	private readonly double cos;
	private readonly double sin;

	public Rotation2d()
	{
		this.value = 0.0;
		this.cos = 0.0;
		this.sin = 0.0;
	}

    /// <summary>
    /// Constructs a Rotation2d with the given radian value.
    /// </summary>
    /// <param name="value">The value of the angle in radians</param>
    public Rotation2d(double value)
	{
		this.value = value;
		this.cos = Math.Cos(value);
		this.sin = Math.Sin(value);
	}

	/// <summary>
	/// 
	/// </summary>
	/// <param name="x">Cosine component</param>
	/// <param name="y">Sine component</param>
	public Rotation2d(double x, double y)
	{
		double magnitude = MathUtils.Hypot(x, y);
		if (magnitude > 1e-6)
		{
			sin = y / magnitude;
			cos = x / magnitude;
		} else
		{
			sin = 0.0;
			cos = 1.0;
		}
		value = Math.Atan2(cos, sin);
	}
	
	public static Rotation2d FromDegrees(double degrees)
	{
		return new Rotation2d(MathUtils.ToRadians(degrees));
	}

	public Rotation2d Minus(Rotation2d other)
	{
		return RotateBy(other.UnaryMinus());
	}

	public Rotation2d Plus(Rotation2d other)
	{
		return RotateBy(other);
	}

	public Rotation2d RotateBy(Rotation2d other)
	{
		return new Rotation2d(
			cos * other.cos - sin * other.sin, cos * other.sin + sin * other.cos);
	}

	public Rotation2d UnaryMinus()
	{
		return new Rotation2d(-value);
	}

	public static Rotation2d FromRotations(double rotations)
	{
		return new Rotation2d(Units.RotationsToRadians(rotations));
	}

	public double GetDegrees()
	{
		return Units.RadiansToDegrees(value);
	}

	public double GetRotations()
	{
		return Units.RadiansToRotations(value);
	}

	public double GetCos()
	{
		return cos;
	}

	public double GetSin()
	{
		return sin;
	}

	public double GetRadians()
	{
		return value;
	}
}

