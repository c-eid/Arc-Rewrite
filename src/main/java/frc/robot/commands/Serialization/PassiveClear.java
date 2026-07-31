// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Serialization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Serializer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassiveClear extends Command {
  /** Creates a new PassiveClear. */
  s_Serializer s_Serializer;
  public PassiveClear(s_Serializer s_Serializer) {
    addRequirements(s_Serializer);
    this.s_Serializer = s_Serializer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Serializer.setDiffVoltage(-3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Serializer.setDiffVoltage(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
