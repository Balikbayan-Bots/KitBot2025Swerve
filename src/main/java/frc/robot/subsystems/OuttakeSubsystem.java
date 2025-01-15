package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase{ 
     /** Creates a new ExampleSubsystem. */
  private SparkFlex m_outtakeMotor;
  public OuttakeSubsystem() {
      m_outtakeMotor = new SparkFlex(20, MotorType.kBrushless);
     ConfigMotor(m_outtakeMotor);
  }

  private void ConfigMotor(SparkFlex motor){
    //bagel
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(false);
    motor.configure(config, null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Grabs outtake value from command and passes to class
  public void set(double speed) {
    m_outtakeMotor.set(speed);
  }

  public void stop() {
    m_outtakeMotor.set(0);
  }
}
