package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem {
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private double x = limelight.getEntry("tx").getDouble(0);
    private double y = limelight.getEntry("ty").getDouble(0);
    private double a = limelight.getEntry("ta").getDouble(0);



    
}
