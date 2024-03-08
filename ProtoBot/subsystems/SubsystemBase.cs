using System;
namespace ProtoBot.subsystems
{
	public interface ISubsystemBase
	{
		//TODO: Make this run periodically with CommandScheduler
		void Periodic();
	}
}

