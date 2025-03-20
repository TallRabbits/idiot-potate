package frc.robot;

public class Constants {
    public static final String CANBUS_CANIVORE = "drivetrain";
    public static final String CANBUS_RIO = "rio";
    
    public static final String[] LIMELIGHT_NAMES = {"limelight_elevator", "limelight_swerve"};

    /* Elevator IDs */
    public static final int ELEVATOR_LEADER_ID = 1;
    public static final int ELEVATOR_FOLLOWER_ID = 2;

    /* Intake IDs */
    public static final int INTAKE_ROLLER_ID = 3;
    public static final int INTAKE_PIVOT_ID = 4;

    /* Climber IDs */
    public static final int CLIMBER_LEADER_ID = 5;
    public static final int CLIMBER_FOLLOWER_ID = 6;

    /* Pooper IDs */
    public static final int CORAL_ROLLER_ID = 7;
    public static final int ALGAE_ROLLER_ID = 8;
    public static final int POOPER_PIVOT_ID = 9;
    public static final int CORAL_SENSOR_ID = 10;

    /* Supply Current Limit */
    public static final int SUPPLY_CURRENT_LIMIT = 40;

    public static enum RobotStates {
        L1,
        L2,
        L3,
        L4,
        DEALGAE_UPPER,
        DEALGAE_LOWER,
        BARGE,
        INTAKE_TRANSFER,
        CORAL_STATION
    }
}