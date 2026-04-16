// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aiming;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Hood;
import frc.robot.util.u_Dist;
import frc.robot.util.u_Lut;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodTrack extends Command {
  /** Creates a new HoodTrack. */

  u_Dist u_Dist;
  s_Hood hood;

  public HoodTrack(s_Hood hood, u_Dist u_Dist) {
    addRequirements(hood);

    this.hood = hood;
    this.u_Dist = u_Dist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setDegrees(u_Lut.getAngleFrom(u_Dist.getDist(4).in(Feet)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
