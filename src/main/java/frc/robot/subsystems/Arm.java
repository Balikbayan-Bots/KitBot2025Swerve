// Copyright (c) bagel and other bagel contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the bagel BSD (Bagel Sound Design) license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.AbsoluteSensorDiscontinuityPointValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX m_ArmMotor;
  private CANcoder m_Encoder;
  private final MotionMagicVoltage m_positionOut;

  private static double m_angle = 20.0;

  private static double m_reference = 0.0;
  /** Creates a new Arm. */
  public Arm() {
    m_ArmMotor = new TalonFX(30);
    m_Encoder = new CANcoder(31);
    
    // Creating new control modes
    m_positionOut = new MotionMagicVoltage(0.0).withSlot(0);
    
    
    //ConfigureArm(m_ArmMotor.getConfigurator(), m_Encoder.getConfigurator());
  }

 /*  private void ConfigureArm(TalonFXConfigurator motorConfig, CANcoderConfigurator EncoderConfig){
  CANcoderConfiguration newEncoderConfig = new CANcoderConfiguration();
  if(EncoderConfig != null) {
    // Set feedback sensor to CANCoder
    FeedbackConfigs feedback = newEncoderConfig.Feedback;
    feedback.FeedbackRemoteSensorID = EncoderConfig.getDeviceID();
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    //Configing the arm encoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPointValue.Signed_PlusMinusHalf;

  
  encoderConfig.apply(newEncoderConfig);

  
  TalonFXConfiguration newMotorConfig = new TalonFXConfiguration();
  // Set soft limits
  var limits = newMotorConfig.SoftwareLimitSwitch;
  limits.ForwardSoftLimitEnable = true;
  limits.ForwardSoftLimitThreshold = kArmLimits.forwardLimit();
  limits.ReverseSoftLimitEnable = true;
  limits.ReverseSoftLimitThreshold = kArmLimits.reverseLimit();

  // Configure idle mode and polarity
  var output = newMotorConfig.MotorOutput;
  output.Inverted = InvertedValue.CounterClockwise_Positive;
  output.NeutralMode = NeutralModeValue.Brake;

  // Set max voltage
  var voltage = newMotorConfig.Voltage;
  voltage.PeakForwardVoltage = 16;
  voltage.PeakReverseVoltage = -16;

  // Set ramp period (0.02 - 0.05 secs)
  var ramp = newMotorConfig.ClosedLoopRamps;
  ramp.VoltageClosedLoopRampPeriod = 0.05;

  // Set current limits
  var current = newMotorConfig.CurrentLimits;
  current.StatorCurrentLimit = kArmLimits.statorLimit();
  current.StatorCurrentLimitEnable = true;
  current.SupplyCurrentLimit = kArmLimits.supplyLimit();
  current.SupplyCurrentLimitEnable = true;

  // Configure PID in Slot 0
  Slot0Configs slot0 = newMotorConfig.Slot0;
  slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
  slot0.kP = 175.0;
  slot0.kI = 0.0;
  slot0.kD = 0.0;
  slot0.kS = 0.2;
  slot0.kG = 0.5;
  slot0.kV = 0.0;

  // Configuring MotionMagic
  var motionMagic = newMotorConfig.MotionMagic;
  motionMagic.MotionMagicAcceleration = 1500.0;
  motionMagic.MotionMagicCruiseVelocity = 4000.0;
  motionMagic.MotionMagicJerk = 6000.0;
  motorConfig.apply(newMotorConfig);
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    m_ArmMotor.set(speed);
  }

  public void stop() {
    m_ArmMotor.set(0);
  }
}
