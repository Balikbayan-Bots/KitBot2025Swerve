package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LimelightSubsystem extends SubsystemBase{
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private double x;
    private double y = limelight.getEntry("ty").getDouble(0);
    private double a;

    private double x_kP = 0.05D;
    private double a_kP = 0.15D;



    public double alignTx(){
        double output = x*x_kP;
        return output;
    }

    public double alignTa() {
        double error = 5.0D - a;
        return error*a_kP;
    }

    @Override
    public void periodic() {
        x = limelight.getEntry("tx").getDouble(0);
        a = limelight.getEntry("ta").getDouble(0);
        SmartDashboard.putNumber("Limelight Tx Output", alignTx());
        SmartDashboard.putNumber("Limelight Ta Output", alignTa());

    }
    
}
