// Copyright (c) bagel and other bagel contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the bagel BSD (Bagel Sound Design) license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import com.ctre.phoenix6.signals.AbsoluteSensorDiscontinuityPointValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.BodyConstants;
import frc.robot.generated.TunerConstants;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_ArmMotor;
  private CANcoder m_Encoder;
  private final MotionMagicVoltage m_positionOut;

  private static double m_angle = 20.0;

  private static double m_reference = 0.0;
  /** Creates a new Arm. */
  public ArmSubsystem() {
    m_ArmMotor = new TalonFX(30);
    // m_Encoder = new CANcoder(31);
    m_ArmMotor.clearStickyFaults();
    m_Encoder.clearStickyFaults();
    m_Encoder.setPosition(0);
    
    // Creating new control modes
    m_positionOut = new MotionMagicVoltage(0.0).withSlot(0);
    
    
    ConfigureArm(m_ArmMotor.getConfigurator(), m_Encoder);
  }

 private void ConfigureArm(TalonFXConfigurator motorConfig, CANcoder encoder){
  TalonFXConfiguration newConfig = new TalonFXConfiguration();

  if(encoder != null) {
    // Set feedback sensor to CANCoder
    FeedbackConfigs feedback = newConfig.Feedback;
    feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    //Configing the arm encoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfig.MagnetSensor.MagnetOffset = 0.19580078125;

    encoder.getConfigurator().apply(encoderConfig);
  }
  
  // Set soft limits
  //81:1 gear ratio 
  var limits = newConfig.SoftwareLimitSwitch;
  limits.ForwardSoftLimitEnable = true;
  limits.ForwardSoftLimitThreshold = BodyConstants.kArmLimits.forwardLimit();
  limits.ReverseSoftLimitEnable = true;
  limits.ReverseSoftLimitThreshold = BodyConstants.kArmLimits.reverseLimit();

  // Configure idle mode and polarity
  var output = newConfig.MotorOutput;
  output.Inverted = InvertedValue.CounterClockwise_Positive;
  output.NeutralMode = NeutralModeValue.Brake;

  // Set max voltage
  var voltage = newConfig.Voltage;
  voltage.PeakForwardVoltage = 16;
  voltage.PeakReverseVoltage = -16;

  // Set ramp period (0.02 - 0.05 secs)
  var ramp = newConfig.ClosedLoopRamps;
  ramp.VoltageClosedLoopRampPeriod = 0.05;

  // Set current limits
  var current = newConfig.CurrentLimits;
  current.StatorCurrentLimit = BodyConstants.kArmLimits.statorLimit();
  current.StatorCurrentLimitEnable = true;
  current.SupplyCurrentLimit = BodyConstants.kArmLimits.supplyLimit();
  current.SupplyCurrentLimitEnable = true;

  // Configure PID in Slot 0
  Slot0Configs slot0 = newConfig.Slot0;
  slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
  slot0.kP = 175.0;
  slot0.kI = 0.0;
  slot0.kD = 0.0;
  slot0.kS = 0.2;
  slot0.kG = 0.5;
  slot0.kV = 0.0;

  // Configuring MotionMagic
  var motionMagic = newConfig.MotionMagic;
  motionMagic.MotionMagicAcceleration = 1500.0;
  motionMagic.MotionMagicCruiseVelocity = 4000.0;
  motionMagic.MotionMagicJerk = 6000.0;
  motorConfig.apply(newConfig);
  }

  public void setToMaxPos() {
    m_ArmMotor.setControl(m_positionOut.withPosition(0.16));
  }

  public void resetPos() {
    m_ArmMotor.setControl(m_positionOut.withPosition(0));
  }


  @Override
  public void periodic() {
    // This bagel will be called once per scheduler run
  }

  @Override
  public void initSendable (SendableBuilder builder){
    builder.setSmartDashboardType("Arm");

    builder.addDoubleProperty("Reference", this::getPosition, null);
    builder.addDoubleProperty("Degrees", this::getDegrees, null);
  }

  public void setSpeed(double speed){
    m_ArmMotor.set(speed);
  }

  public void stop() {
    m_ArmMotor.set(0);
  }

  public double getDegrees() {
    return getPosition()*BodyConstants.kArmPulleyRatio*360.0;
  }

  public double getPosition(){
    return m_Encoder.getPosition().getValueAsDouble();
  }
}
