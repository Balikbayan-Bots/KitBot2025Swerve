package main.java.frc.robot.subsystems;

public class ShooterSubsystem {
     /** Creates a new ExampleSubsystem. */
  private CANSparkMax shooter = new CANSparkMax(20, MotorType.kBrushless);

  public ShooterSubsystem() {
          // clear controller settings
    shooter.restoreFactoryDefaults();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Grabs shooter value from command and passes to class
  public void set(double ShooterVal) {
    shooter.set(ShooterVal);
  }
}
