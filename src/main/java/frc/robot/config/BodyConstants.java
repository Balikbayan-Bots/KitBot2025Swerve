// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

/** Add your docs here. */
public class BodyConstants {
    public static final Limits kArmLimits = new Limits(80.0, 45.0, 0, 0);
    public static final double kArmPulleyRatio = 18.0/54.0;
    public record Limits(double statorLimit, double supplyLimit, double forwardLimit, double reverseLimit) {};
}
