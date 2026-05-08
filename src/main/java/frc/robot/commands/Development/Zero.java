// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Development;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Hood;
import frc.robot.subsystems.s_Intake;
import frc.robot.subsystems.s_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Zero extends Command {

  s_Turret s_Turret;
  s_Hood s_Hood;
  s_Intake s_Intake;

  public Zero(s_Turret s_Turret, s_Hood s_Hood, s_Intake s_Intake) {
    this.s_Turret = s_Turret;
    this.s_Hood = s_Hood;
    this.s_Intake = s_Intake;

    addRequirements(s_Turret, s_Hood, s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Turret.setDegrees(0);
    s_Hood.setDegrees(0);
    s_Intake.setDegrees(115);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
