using System;
using CTRE.Phoenix6.Signals;

namespace ProtoBot;

	public class Constants
	{
		public class SwerveConstants
		{
			public static readonly double driveGearRatio = 0.0;
            public static readonly double angleGearRatio = 0.0;

			public static readonly InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            public static readonly InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
            public static readonly SensorDirectionValue canCoderSensorDirection = SensorDirectionValue.Clockwise_Positive;

            public static readonly double driveKP = 0.0;
            public static readonly double driveKI = 0.0;
            public static readonly double driveKD = 0.0;

            public static readonly double angleKP = 0.0;
            public static readonly double angleKI = 0.0;
            public static readonly double angleKD = 0.0;
    }
}

