using System;
namespace ProtoBot.utils.math.geometry;

public class Pose2d
{
	private readonly Translation2d translation;
	private readonly Rotation2d rotation;

	public Pose2d()
	{
		translation = new();
		rotation = new();
	}

	public Pose2d(Translation2d translation, Rotation2d rotation)
	{
		this.translation = translation;
		this.rotation = rotation;
	}

	public Pose2d(double x, double y, Rotation2d rotation)
	{
		this.translation = new Translation2d(x, y);
		this.rotation = rotation;
	}

	public Pose2d Plus(Transform2d other)
	{
		return TransformBy(other);
	}

	public Pose2d TransformBy(Transform2d other)
	{
		return new Pose2d(
			translation.Plus(other.GetTranslation().RotateBy(rotation)),
			other.GetRotation().Plus(rotation));
	}

	public Pose2d Exp(Twist2d twist)
	{
		double dx = twist.dx;
		double dy = twist.dy;
		double dtheta = twist.dtheta;

		double sinTheta = Math.Sin(dtheta);
		double cosTheta= Math.Cos(dtheta);

		double s;
		double c;
		if (Math.Abs(dtheta) < 1E-9)
		{
			s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
			c = 0.5 * dtheta;
		} else
		{
			s = sinTheta / dtheta;
			c = (1 - cosTheta) / dtheta;
		}
		var transform =
			new Transform2d(
				new Translation2d(dx * s - dy * c, dx * c + dy * s),
				new Rotation2d(cosTheta, sinTheta));
		return this.Plus(transform);
	}

	public Translation2d GetTranslation()
	{
		return translation;
	}

	public Pose2d RelativeTo(Pose2d other)
	{
		var transform = new Transform2d(other, this);
		return new Pose2d(transform.GetTranslation(), transform.GetRotation());
	}

	public Twist2d Log(Pose2d end)
	{
		var transform = end.RelativeTo(this);
		var dtheta = transform.GetRotation().GetRadians();
		var halfDtheta = dtheta / 2.0;

		var cosMinusOne = transform.GetRotation().GetCos() - 1;

		double halfThetaByTanOfHalfDtheta;
		if (Math.Abs(cosMinusOne) < 1E-9)
		{
			halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
		} else
		{
			halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.GetRotation().GetSin()) / cosMinusOne;
		}

		Translation2d translationPart =
			transform
			.GetTranslation()
			.RotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta))
			.Times(MathUtils.Hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

		return new Twist2d(translationPart.GetX(), translationPart.GetY(), dtheta);
	}

	public Rotation2d GetRotation()
	{
		return rotation;
	}
}

