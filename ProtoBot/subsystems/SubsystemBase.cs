namespace ProtoBot.subsystems;

public abstract class SubsystemBase
{
    protected PeriodicTimer Timer;
    protected CancellationTokenSource Cts = new();

    protected SubsystemBase()
    {
        Timer = new PeriodicTimer(TimeSpan.FromMilliseconds(20)); // 20ms
        Start();
    }

    public abstract void Periodic();
    public abstract bool End();

    public void Start()
    {
        Task.Run(async () =>
        {
            try
            {
                while (await Timer.WaitForNextTickAsync(Cts.Token))
                {
                    Periodic();

                    if (End())
                    {
                        Cts.Cancel();
                        break;
                    }
                }
            } catch (OperationCanceledException)
            {

            }
        });
    }
}


