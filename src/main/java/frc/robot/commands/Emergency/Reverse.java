// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Emergency;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Belt;
import frc.robot.subsystems.s_Intake;
import frc.robot.subsystems.s_Serializer;
import frc.robot.subsystems.s_Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Reverse extends Command {
  /** Creates a new Reverse. */
  s_Shooter s_Shooter;
  s_Belt s_Index;
  s_Serializer s_Serializer;
  s_Intake s_Intake;

  public Reverse(s_Shooter s_Shooter, s_Belt s_Index, s_Serializer s_Serializer, s_Intake s_Intake) {
    addRequirements(s_Index, s_Intake, s_Shooter, s_Serializer);

    this.s_Shooter = s_Shooter;
    this.s_Index = s_Index;
    this.s_Serializer = s_Serializer;
    this.s_Intake = s_Intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Index.setIndexRpm(-3000);
    s_Shooter.setRPM(-2000);
    s_Intake.setSpeed(-1);
    s_Serializer.setVoltage(-12);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      s_Index.setIndexRpm(0);
    s_Shooter.setRPM(0);
    s_Intake.setSpeed(0);
    s_Serializer.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
