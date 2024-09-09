namespace AWSIM_Script.Object
{
    // delay based on distance from ego to npc
    public class NPCDelayDistance : INPCSpawnDelay
    {
        public DelayedAction ActionDelayed { get; set; }
        public float Distance { get; set; }
        

        // delay until the distance between Ego and NPC reach $distance
        public static NPCDelayDistance DelayMove(float distance)
        {
            return new NPCDelayDistance()
            {
                Distance = distance,
                ActionDelayed = DelayedAction.MOVING
            };
        }
    }
}