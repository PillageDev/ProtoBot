using System;
using ProtoBot.utils.math.geometry;
using ProtoBot.utils.math.geometry;

namespace ProtoBot.utils.math.kinematics;

public class SwerveDriveKinematics
{
	private readonly int numModules;
	private readonly Translation2d[] modules;
	private Rotation2d[] moduleHeadings;
	private Translation2d prevCoR = new();

        public SwerveDriveKinematics(params Translation2d[] moduleTranslationMeters)
	{
		if (moduleTranslationMeters.Length <= 3)
		{
			throw new("Where are the other modules?");
		}

	}
}

