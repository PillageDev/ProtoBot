namespace ProtoBot.utils.math.geometry;

public class Transform2d
{
    private readonly Translation2d translation;
    private readonly Rotation2d rotation;
    
    public Transform2d(Translation2d translation, Rotation2d rotation)
    {
        this.translation = translation;
        this.rotation = rotation;
    }
    
    public Transform2d(Pose2d initial, Pose2d last)
    {
        translation = last.GetTranslation().Minus(initial.GetTranslation()).RotateBy(initial.GetRotation().UnaryMinus());
        rotation = last.GetRotation().Minus(initial.GetRotation());
    }
    
    public Translation2d GetTranslation()
    {
        return translation;
    }
    
    public Rotation2d GetRotation()
    {
        return rotation;
    }
}