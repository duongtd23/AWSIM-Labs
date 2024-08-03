using System;
namespace AWSIM_Script.Object
{
    public enum DelayKind
    {
        FROM_BEGINNING,           //default
        UNTIL_EGO_ENGAGE,
        UNTIL_EGO_MOVE,
        NONE
    }
    public enum DelayedAction
    {
        SPAWNING, // spawning NPC is delayed
        MOVING    // spawn NPC as it is, but delay its movement
    }
    public class NPCSpawnDelay
    {
        public float DelayAmount { get; set; }
        public DelayKind DelayType { get; set; }
        public DelayedAction ActionDelayed { get; set; }

        public NPCSpawnDelay()
        {
            DelayType = DelayKind.NONE;
        }

        // Delay `delay` seconds from the beginning before spawning the NPC
        public static NPCSpawnDelay DelaySpawn(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = DelayKind.FROM_BEGINNING,
                ActionDelayed = DelayedAction.SPAWNING
            };
        }
        // Spawn NPC, but delay `delay` seconds from the beginning before letting it move
        public static NPCSpawnDelay DelayMove(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = DelayKind.FROM_BEGINNING,
                ActionDelayed = DelayedAction.MOVING
            };
        }

        // Delay spawning NPC until Ego got engaged (in seconds).
        // E.g., if the passed param (`delay`) is 2,
        // 2 seconds after Ego engaged, the NPC will be spawned
        // If `delay` is 0, NPC will be spawned at the same time when Ego engaged.
        public static NPCSpawnDelay DelaySpawnUntilEgoEngaged(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = DelayKind.UNTIL_EGO_ENGAGE,
                ActionDelayed = DelayedAction.SPAWNING
            };
        }
        // Delay moving NPC until the Ego vehicle got engaged (in seconds).
        public static NPCSpawnDelay DelayMoveUntilEgoEngaged(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = DelayKind.UNTIL_EGO_ENGAGE,
                ActionDelayed = DelayedAction.MOVING
            };
        }

        // Delay spawning NPC until Ego moves (in seconds)
        // E.g., if the passed param (`delay`) is 2,
        // 2 seconds after Ego moves, the NPC will be spawned
        public static NPCSpawnDelay DelaySpawnUntilEgoMove(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = DelayKind.UNTIL_EGO_MOVE,
                ActionDelayed = DelayedAction.SPAWNING
            };
        }

        // Delay moving NPC until Ego moves.
        // Don't set `delay` value to 0 as it may cause the NPC and the Ego never move.
        // In such a case, use DelayMoveUntilEgoEngaged instead
        public static NPCSpawnDelay DelayMoveUntilEgoMove(float delay)
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = delay,
                DelayType = DelayKind.UNTIL_EGO_MOVE,
                ActionDelayed = DelayedAction.MOVING
            };
        }

        public static NPCSpawnDelay DummyDelay()
        {
            return new NPCSpawnDelay()
            {
                DelayAmount = 0,
                DelayType = DelayKind.NONE,
                ActionDelayed = DelayedAction.SPAWNING
            };
        }

        public override bool Equals(object obj)
        {
            if (obj is NPCSpawnDelay)
            {
                var obj2 = (NPCSpawnDelay)obj;
                return this.DelayType == obj2.DelayType &&
                    this.DelayAmount == obj2.DelayAmount;
            }
            return base.Equals(obj);
        }
    }
}

