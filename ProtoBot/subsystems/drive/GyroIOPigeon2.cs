using CTRE.Phoenix6.Hardware.Core;
using ProtoBot.utils;
using ProtoBot.utils.math.geometry;

namespace ProtoBot.subsystems.drive;

public class GyroIOPigeon2: IGyroIO
{
    public readonly CorePigeon2 pigeon = new(0); //TODO: Get the correct CanID

    public static readonly bool INVERTED = true; //TODO: Check

    public GyroIOPigeon2()
    {
        // TODO: Reset the pigeon
    }

    public void ResetYaw()
    {
        //TODO: Reset the pigeon
    }

    public void UpdateInputs(IGyroIO.GyroIOInputs inputs)
    {
        double[] ypr = new double[3];
        ypr[0] = pigeon.GetYaw().ValueAsDouble;
        ypr[1] = pigeon.GetPitch().ValueAsDouble;
        ypr[2] = pigeon.GetRoll().ValueAsDouble;

        inputs.connected = true;
        inputs.realYawPosition = Rotation2d.FromDegrees(ypr[0] * (INVERTED ? -1.0 : 1.0)); //TODO: Check if this is accurate
        inputs.yawPosition = inputs.realYawPosition.Minus(inputs.yawOffset);
        inputs.yawVelocityRadPerSec = Units.DegreesToRadians(pigeon.GetAngularVelocityZ().ValueAsDouble);
        inputs.pose = new Rotation3d(MathUtils.ToRadians(ypr[0]), MathUtils.ToRadians(ypr[1]), MathUtils.ToRadians(ypr[2]));
    }
}