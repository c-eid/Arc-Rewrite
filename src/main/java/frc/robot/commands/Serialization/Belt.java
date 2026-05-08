// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Serialization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Belt;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Belt extends Command {
  /** Creates a new Belt. */
  s_Belt s_Belt;

  public Belt(s_Belt s_Belt) {
    this.s_Belt = s_Belt;
    addRequirements(s_Belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Belt.setIndexRpm(4041);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Belt.setIndexRpm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
