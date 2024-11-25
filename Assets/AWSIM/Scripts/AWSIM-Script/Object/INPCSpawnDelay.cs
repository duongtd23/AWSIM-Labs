namespace AWSIM_Script.Object
{
    // abstract for the spawn option
    public interface INPCSpawnDelay
    {
        public DelayedAction ActionDelayed { get; set; }
    }
    
    public enum DelayedAction
    {
        SPAWNING, // spawning NPC is delayed
        MOVING    // spawn NPC as it is, but delay its movement
    }
}