// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Serialization;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.s_Belt;
import frc.robot.subsystems.s_Serializer;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopSerializer extends InstantCommand {
  s_Belt s_Belt;
  s_Serializer s_Serializer;  

  public StopSerializer(s_Belt s_Belt, s_Serializer s_Serializer) {
    this.s_Belt = s_Belt;
    this.s_Serializer = s_Serializer;
    addRequirements(s_Belt, s_Serializer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Belt.setIndexRpm(0);
    s_Serializer.setDiffVoltage(0);
  }
}
