// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Launch;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Shooter;
import frc.robot.util.u_Dist;
import frc.robot.util.u_Lut;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new Shoot. */
  s_Shooter s_Shooter;
  u_Dist u_Dist;

  public Shoot(s_Shooter s_Shooter, u_Dist u_Dist) {
    addRequirements(s_Shooter);

    this.s_Shooter = s_Shooter;
    this.u_Dist = u_Dist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Shooter.setRPM( ()->
        u_Lut.getRpmFrom(
            u_Dist.getDist().in(Feet)));
  }

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
