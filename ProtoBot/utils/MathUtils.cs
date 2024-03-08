using System;
namespace ProtoBot.utils;

public class MathUtils
{
	public static double Hypot(double x, double y)
	{
		return Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2));
	}

	public static double ToRadians(double degrees)
	{
		return degrees * (Math.PI / 180);
	}
}

