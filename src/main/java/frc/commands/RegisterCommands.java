// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.OuttakeSubsystem;

/** Add your docs here. */
public class RegisterCommands {
    private final OuttakeSubsystem m_outtake;
    public RegisterCommands(OuttakeSubsystem outtake) {
    m_outtake = outtake;

    NamedCommands.registerCommand("runOuttake", OuttakeCommands.OuttakeOut(m_outtake));
    NamedCommands.registerCommand("stopOuttake", OuttakeCommands.OuttakeStop(m_outtake));
    }
}
