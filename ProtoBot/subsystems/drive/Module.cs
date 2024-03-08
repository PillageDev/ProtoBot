using System;
using ProtoBot.utils;
using ProtoBot.utils.geometry;

namespace ProtoBot.subsystems.drive
{
	public class Module
	{
		private static readonly double WHEEL_RADIUS = Units.InchesToMeters(2.0); //TODO: Change this
		private static readonly double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;
		public static readonly double ODOMETRY_FREQUENCY = 250.0;

		// Module IO
		// Module IO Inputs
		private readonly int index;

		// PID Controller
		private Rotation2d? angleSetpoint = null;
		private double? speedSetpoint = null;
		private Rotation2d? turnRelativeOffset = null;
		private double lastPositionMeters = 0.0;
		// Swerve module position

		public Module(/* ModuleIO */ int index)
		{
			this.index = index;
		}

		public void updateInputs()
		{
			// io.updateInputs(inputs);
		}

		public void periodic()
		{

		}
	}
}

