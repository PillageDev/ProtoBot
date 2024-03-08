using System;
namespace ProtoBot.utils.math.geometry;

public class Rotation3d
{
	private readonly Quaternion q;

	public Rotation3d()
	{
		this.q = new Quaternion();
	}

        public Rotation3d(double roll, double pitch, double yaw)
	{
		// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
		double cr = Math.Cos(roll * 0.5);
		double sr = Math.Sin(roll * 0.5);

		double cp = Math.Cos(pitch * 0.5);
		double sp = Math.Sin(pitch * 0.5);

		double cy = Math.Cos(yaw * 0.5);
		double sy = Math.Sin(yaw * 0.5);

		this.q =
			new Quaternion(
				cr * cp * cy + sr * sp * sy,
				sr * cp * cy - cr * sp * sy,
				cr * sp * cy + sr * cp * sy,
				cr * cp * sy - sr * sp * cy);
	}
}

