using System;
namespace ProtoBot.utils.math.geometry
{
	public class Twist2d
	{
		public double dx;
		public double dy;
		public double dtheta;

		public Twist2d(double dx, double dy, double dtheta)
		{
			this.dx = dx;
			this.dy = dy;
			this.dtheta = dtheta;
		}
	}
}

