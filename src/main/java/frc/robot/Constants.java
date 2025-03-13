package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Constants {
    public static final String CANivoreName = "canivore";

    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public class ClimbConstants {
        public static final int ClimbId = 10;

        public static final NetworkTable ClimbTable = inst.getTable("Climb");
    }
    public class CoralEndEffectorConstants {
        public static final int RollerId = 11;
        public static final int WristId = 12;
        
        public static final NetworkTable EndEffectorTable = inst.getTable("EndEffector");
    }
    public class CoralIndexerConstants {
        public static final int GroundIntakeId = 13;
        public static final int FlipperId = 14;
        
        public static final NetworkTable IndexerTable = inst.getTable("Indexer");
    }
    public class ElevatorConstants {
        public static final int LeftId = 15;
        public static final int RightId = 16;
        
        public static final NetworkTable ElevatorTable = inst.getTable("Elevator");
    }
}
