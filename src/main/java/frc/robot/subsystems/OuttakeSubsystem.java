package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
    private SparkMax shooterMotor = new SparkMax(20, MotorType.kBrushless);

    public OuttakeSubsystem() {

    }


    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

  // Grabs shooter value from command and passes to class
    public void set(double ShooterVal) {
        shooterMotor.set(ShooterVal);
    }  

}
