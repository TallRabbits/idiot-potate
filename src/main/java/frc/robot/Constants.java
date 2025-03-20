package frc.robot;

public class Constants {
    public static final String CANBUS_CANIVORE = "drivetrain";
    public static final String CANBUS_RIO = "rio";
    
    public static final String[] LIMELIGHT_NAMES = {"limelight-elevato", "limelight-swerve"};

    /* Elevator IDs */
    public static final int ELEVATOR_LEADER_ID = 10;
    public static final int ELEVATOR_FOLLOWER_ID = 11;

    /* Intake IDs */
    public static final int INTAKE_PIVOT_ID = 20;
    public static final int INTAKE_ROLLER_ID = 21;
    
    /* Climber IDs */
    // public static final int CLIMBER_ID = 1;

    /* Pooper IDs */
    public static final int POOPER_PIVOT_ID = 30;
    public static final int CORAL_ROLLER_ID = 31;
    public static final int CORAL_SENSOR_ID = 32;
    public static final int ALGAE_ROLLER_ID = 33;
    
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