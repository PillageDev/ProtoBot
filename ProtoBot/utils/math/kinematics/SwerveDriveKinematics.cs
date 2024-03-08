using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Complex;
using ProtoBot.utils.math.geometry;

namespace ProtoBot.utils.math.kinematics;

public class SwerveDriveKinematics
{
	private readonly Matrix<double> inverseKinematics;
	private readonly Matrix<double> forwardKinematics;

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
		numModules = moduleTranslationMeters.Length;
		Array.Copy(moduleTranslationMeters, modules, numModules);
		moduleHeadings = new Rotation2d[numModules];
		Array.Fill(moduleHeadings, new Rotation2d());
		inverseKinematics = Matrix<double>.Build.Dense(numModules * 2, 3);

		for (int i = 0; i < numModules; i++)
		{
			inverseKinematics.SetRow(i * 2 + 0, new double[] { 1, 0, -modules[i].GetY() });
			inverseKinematics.SetRow(i * 2 + 0, new double[] { 0, 1, +modules[i].GetX() });
		}
		forwardKinematics = inverseKinematics.PseudoInverse();
	}

	public Twist2d ToTwist2d(params SwerveModulePosition[] moduleDeltas)
	{
		if (moduleDeltas.Length != numModules)
		{
			throw new("Number of modules is not consistant with number of module locations provided in the constructor");
		}

		var moduleDeltaMatrix = Matrix<double>.Build.Dense(numModules * 2, 1);

		for (int i = 0; i < numModules; i++)
		{
			var module = moduleDeltas[i];
			moduleDeltaMatrix[i * 2, 0] = module.distanceMeters * module.angle.GetCos(); 
			moduleDeltaMatrix[i * 2 + 1, 0] = module.distanceMeters * module.angle.GetSin();
		}

		var chassisDeltaVector = forwardKinematics.Multiply(moduleDeltaMatrix);
		return new Twist2d(
			chassisDeltaVector[0, 0], chassisDeltaVector[1, 0], chassisDeltaVector[2, 0]);
	}
}

