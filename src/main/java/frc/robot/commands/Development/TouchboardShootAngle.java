// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Development;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Hood;
import frc.robot.subsystems.s_Shooter;
import frc.robot.util.Touchboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TouchboardShootAngle extends Command {
  /** Creates a new Shoot. */
  s_Shooter s_Shooter;
  s_Hood s_Hood;

  public TouchboardShootAngle(s_Shooter s_Shooter, s_Hood s_Hood) {
    addRequirements(s_Shooter, s_Hood);

    this.s_Shooter = s_Shooter;
    this.s_Hood = s_Hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Shooter.setRPM( () -> Touchboard.getDoubleValue("tbRpm"));
    s_Hood.setDegrees( Touchboard.getDoubleValue("tbAngle"));
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
