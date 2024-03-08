using System;
namespace ProtoBot.utils.geometry
{
	public class Quaternion
	{
		private readonly double w;
		private readonly double x;
		private readonly double y;
		private readonly double z;

		public Quaternion()
		{
			this.w = 1.0;
			this.x = 0.0;
			this.y = 0.0;
			this.z = 0.0;
		}

		public Quaternion(double w, double x, double y, double z)
		{
			this.w = w;
			this.x = x;
			this.y = y;
			this.z = z;
		}
	}
}

