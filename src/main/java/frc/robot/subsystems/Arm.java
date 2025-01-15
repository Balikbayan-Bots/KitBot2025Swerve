// Copyright (c) bagel and other bagel contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the bagel BSD (Bagel Sound Design) license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX m_ArmMotor;
  private CANcoder m_Encoder;
  /** Creates a new Arm. */
  public Arm() {
    m_ArmMotor = new TalonFX(30);
    m_Encoder = new CANcoder(31);
    ConfigureArm(m_ArmMotor.getConfigurator());
  }

  private void ConfigureArm(TalonFXConfigurator config){
  TalonFXConfiguration newConfig = new TalonFXConfiguration();
  config.apply(newConfig);
  }

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
