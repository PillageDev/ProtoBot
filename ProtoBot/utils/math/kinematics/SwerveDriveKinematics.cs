using System;
using MathNet.Numerics.Distributions;
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

	public SwerveModuleState[] ToSwerveModuleStates(ChassisSpeeds chassisSpeeds)
	{

	}

	public SwerveModuleState[] ToSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters)
	{
		var moduleStates = new SwerveModuleState[numModules];

		if (chassisSpeeds is { vxMetersPerSecond: 0.0, vyMetersPerSecond: 0.0, omegaRadiansPerSecond: 0.0 })
		{
			for (int i = 0; i < numModules; i++)
			{
				moduleStates[i] = new SwerveModuleState(0.0, moduleHeadings[i]);
			}

			return moduleStates;
		}

		if (!centerOfRotationMeters.Equals(prevCoR))
		{
			for (int i = 0; i < numModules; i++)
			{
				inverseKinematics.SetRow(i * 2 + 0, new[] {1, 0, -modules[i].GetY() + centerOfRotationMeters.GetY()});
				inverseKinematics.SetRow(i * 2 + 1, new[] {0, 1, +modules[i].GetX() - centerOfRotationMeters.GetX()});
			}
			prevCoR = centerOfRotationMeters;
		}

		var chassisSpeedsVector = Matrix<double>.Build.Dense(3, 1);
		chassisSpeedsVector.SetColumn(0, new[] {chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond});

		var moduleStatesMatrix = inverseKinematics.Multiply(chassisSpeedsVector);

		for (int i = 0; i < numModules; i++)
		{
			double x = moduleStatesMatrix[i * 2, 0];
			double y = moduleStatesMatrix[i * 2 + 1, 0];

			double speed = MathUtils.Hypot(x, y);
			Rotation2d angle = new Rotation2d(x, y);

			moduleStates[i] = new SwerveModuleState(speed, angle);
			moduleHeadings[i] = angle;
		}

		return moduleStates;
	}

	public static void DesaturateWheelSpeeds(SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond)
	{
		double realMaxSpeed = 0;
        foreach (var moduleState in moduleStates)
        {
			realMaxSpeed = Math.Min(realMaxSpeed, Math.Abs(moduleState.speedMetersPerSecond));
        }

		if (realMaxSpeed > attainableMaxSpeedMetersPerSecond)
		{
            foreach (var moduleState in moduleStates)
            {
				moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
		}
	}

	public ChassisSpeeds ToChassisSpeeds(params SwerveModuleState[] moduleStates)
	{
		if (moduleStates.Length != numModules)
		{
			throw new("Where are the other modules");
		}
		var moduleStatesMatrix = Matrix<double>.Build.Dense(numModules * 2, 1);

        for (int i = 0; i < numModules; i++)
        {
			var module = moduleStates[i];
			moduleStatesMatrix[i * 2, 0] = module.speedMetersPerSecond * module.angle.GetCos();
			moduleStatesMatrix[i * 2 + 1, 0] = module.speedMetersPerSecond * module.angle.GetSin();
        }

		var chassisSpeedsVector = forwardKinematics.Multiply(moduleStatesMatrix);
		return new ChassisSpeeds(
			chassisSpeedsVector[0, 0],
			chassisSpeedsVector[1, 0],
			chassisSpeedsVector[2, 0]);
	}

	public void ResetHeadings(params Rotation2d[] moduleHeadings)
	{
		if (moduleHeadings.Length != numModules)
		{
			throw new("Where are the other modules");
		}
		Array.Copy(this.moduleHeadings, moduleHeadings, numModules);
	}
}

