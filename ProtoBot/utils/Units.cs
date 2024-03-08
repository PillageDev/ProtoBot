using System;
namespace ProtoBot.utils
{
	public class Units
	{
		public static double FeetToMeters(double feet)
		{
			return feet * 0.3048;
        }

		public static double InchesToMeters(double inches)
		{
			return inches / 39.37;
        }

		public static double DegreesToRadians(double degrees)
		{
            return degrees * (Math.PI / 180);
        }

		public static double RotationsToDegrees(double rotations)
		{
			return rotations * 180;
		}

		public static double RotationsToRadians(double rotations)
		{
			return rotations * 2 * Math.PI;
		}

		public static double RadiansToDegrees(double radians)
		{
			return radians * (180 / Math.PI);
		}

		public static double RadiansToRotations(double radians)
		{
			return radians / (Math.PI * 2);
		}
    }
}

