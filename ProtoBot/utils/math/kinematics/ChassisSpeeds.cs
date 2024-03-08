using ProtoBot.utils.math.geometry;

namespace ProtoBot.utils.math.kinematics;

public class ChassisSpeeds {
    public double vxMetersPerSecond;
    public double vyMetersPerSecond;
    public double omegaRadiansPerSecond;
    
    public ChassisSpeeds()
    {
        this.vxMetersPerSecond = 0.0;
        this.vyMetersPerSecond = 0.0;
        this.omegaRadiansPerSecond = 0.0;
    }
    
    public ChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond)
    {
        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }
    
    public static ChassisSpeeds Discretize(ChassisSpeeds continuousSpeeds, double dtSeconds)
    {
        return Discretize(
            continuousSpeeds.vxMetersPerSecond,
            continuousSpeeds.vyMetersPerSecond,
            continuousSpeeds.omegaRadiansPerSecond,
            dtSeconds);
    }
    
    public static ChassisSpeeds Discretize(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, double dtSeconds)
    {
        var desiredDeltaPose = new Pose2d(vxMetersPerSecond * dtSeconds, vyMetersPerSecond * dtSeconds, new Rotation2d(omegaRadiansPerSecond * dtSeconds));
        var twist = new Pose2d().Log(desiredDeltaPose);
        return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
    }
}