// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Emergency;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Shooter;
import frc.robot.util.u_Lut;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrenchShot extends Command {
  /** Creates a new TrenchShot. */
  s_Shooter s_Shooter;

  public TrenchShot(s_Shooter s_Shooter) {
    addRequirements(s_Shooter);
    this.s_Shooter = s_Shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.setRPM(u_Lut.getRpmFrom(10.5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
