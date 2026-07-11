// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Seconds;


import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.s_Shooter;
import frc.robot.subsystems.sim.PhysicsSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override 
  public void robotInit(){
    DataLogManager.start();
    SignalLogger.start();

    led.enablePWM(3);
    led.setPWMRate(1000);

    addPeriodic(() -> prematchScheduler(), Seconds.of(1));
    addPeriodic(()-> m_robotContainer.u_Dist.periodic(), getPeriod());    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("uDist/GoalDistance", m_robotContainer.u_Dist.getDist().in(Feet));
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
  DigitalOutput led = new DigitalOutput(6);
  boolean runScheduler = true;

  @Override
  public void disabledInit() {
    runScheduler = true;

  }

  @Override
  public void disabledPeriodic() {}

  //Partition 1 : Questnav, Partition 2: Manual, Partition 3: Auto Start
  public void prematchScheduler(){
    Command ledSetter = Commands.none();

    if(!runScheduler){
      led.updateDutyCycle(1);

      return;
    }

    if(m_robotContainer.s_QNav.getConnected()){
      ledSetter = ledSetter.andThen(
        Commands
        .run(()->{})
        .beforeStarting(() -> {led.updateDutyCycle(11);})
        .finallyDo(()->{led.updateDutyCycle(3);})
        .withTimeout(0.15)
      );
    } else {
      ledSetter = ledSetter.andThen(
        Commands
        .run(()->{})
        .beforeStarting(() -> {led.updateDutyCycle(17);})
        .finallyDo(()->{led.updateDutyCycle(3);})
        .withTimeout(0.15)
      );
    }

    if(m_robotContainer.s_QNav.getStartPoseStatus().equals("NOT SET")){
      ledSetter = ledSetter.andThen(
        Commands
        .run(()->{})
        .beforeStarting(() -> {led.updateDutyCycle(13);})
        .finallyDo(()->{led.updateDutyCycle(3);})
        .withTimeout(0.15)
      );
    } else {
      ledSetter = ledSetter.andThen(
        Commands
        .run(()->{})
        .beforeStarting(() -> {led.updateDutyCycle(19);})
        .finallyDo(()->{led.updateDutyCycle(3);})
        .withTimeout(0.15)
      );
    }

    if(m_robotContainer.s_QNav.getConnected()){
      ledSetter = ledSetter.andThen(
        Commands
        .run(()->{})
        .beforeStarting(() -> {led.updateDutyCycle(15);})
        .finallyDo(()->{led.updateDutyCycle(3);})
        .withTimeout(0.15)
      );
    } else {
      ledSetter = ledSetter.andThen(
        Commands
        .run(()->{})
        .beforeStarting(() -> {led.updateDutyCycle(21);})
        .finallyDo(()->{led.updateDutyCycle(3);})
        .withTimeout(0.15)
      );
    }

    CommandScheduler.getInstance().schedule(ledSetter);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    runScheduler = false;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    runScheduler = false;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      m_robotContainer.s_Shooter.removeDefaultCommand();
      m_robotContainer.s_Serializer.removeDefaultCommand();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
