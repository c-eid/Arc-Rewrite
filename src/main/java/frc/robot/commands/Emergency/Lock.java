// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Emergency;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Hood;
import frc.robot.subsystems.s_Turret;
import frc.robot.util.u_Lut;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lock extends Command {
  /** Creates a new Lock. */
  s_Turret s_Turret;
  s_Hood s_Hood;

  public Lock(s_Turret s_Turret, s_Hood s_Hood) {
    addRequirements(s_Hood, s_Turret);
    this.s_Turret = s_Turret;
    this.s_Hood = s_Hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Turret.setDegrees(90);
    s_Hood.setDegrees(u_Lut.getAngleFrom(10.5));
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
