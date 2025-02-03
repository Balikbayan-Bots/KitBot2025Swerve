// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.OuttakeSubsystem;

/** Add your docs here. */
public class OuttakeCommands {
    public static Command OuttakeOut(OuttakeSubsystem Outtake){
        return new RunCommand(()->{Outtake.set(.3);}, Outtake);
    }
    public static Command OuttakeIn(OuttakeSubsystem Outtake){
        return new RunCommand(()->{Outtake.set(-.3);}, Outtake);
    }
    public static Command OuttakeStop(OuttakeSubsystem Outtake){
        return new RunCommand(()->{Outtake.stop();}, Outtake);
    }
}
