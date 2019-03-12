/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import javax.swing.text.html.HTMLDocument.Iterator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.model.AutonomousLibrary;
import frc.robot.model.GameToolStateMachine;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Vision visionSubsystem;
  public static OI oi;
  public static DriveSubsystemNeo driveSubsystem;
  public static Elevator elevator;
  public static Flower flower;
  public static Switcher switcher;
  public static GameToolStateMachine gameToolStateMachine;
  public static Climber climber;
  public static FMS fms;
  public static Cargo cargo;

  DriveCommand driveCommand;
  ToggleFlowerExtendCommand extenderCommand;
  ToggleFlowerCommand flowerCommand;
  ReverseCommand reverseCommand;
  EncoderResetCommand encoderResetCommand;
  SwitcherUpCommand switcherUpCommand;
  SwitcherDownCommand switcherDownCommand;
  SwitcherEncoderResetCommand switcherEncoderResetCommand;
  GameToolIncrementCommand gameToolIncrementCommand;
  GameToolDecrementCommand gameToolDecrementCommand;
  GameToolSwapCommand gameToolSwapCommand;
  GameToolFlowerCommand gameToolFlowerCommand;
  public static ClimbCommandGroup climbCommandGroup;
  ClimbCancelCommand climbCancelCommand;
  ElevatorControl elevatorControl;
  SwitcherControl switchControl;
  CargoOutCommand cargoOutCommand;
  CargoOutCommand cargoOutLeftCommand;
  CargoOutCommand cargoOutRightCommand;
  CargoInCommand cargoInCommand;
  
  CANSparkMax SPARK;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    
    oi = new OI();
    driveSubsystem = new DriveSubsystemNeo();
    //visionSubsystem = new Vision();
    elevator = new Elevator();
    flower = new Flower();
    switcher = new Switcher();
    gameToolStateMachine = new GameToolStateMachine();
    climber = new Climber();
    fms = new FMS();
    cargo = new Cargo();


    driveCommand = new DriveCommand();
    extenderCommand = new ToggleFlowerExtendCommand();
    reverseCommand = new ReverseCommand();
    encoderResetCommand = new EncoderResetCommand(5);
    switcherUpCommand = new SwitcherUpCommand();
    switcherDownCommand = new SwitcherDownCommand();
    switcherEncoderResetCommand = new SwitcherEncoderResetCommand(5);
    gameToolIncrementCommand = new GameToolIncrementCommand();
    gameToolDecrementCommand = new GameToolDecrementCommand();
    gameToolSwapCommand = new GameToolSwapCommand();
    gameToolFlowerCommand = new GameToolFlowerCommand();
    climbCommandGroup = new ClimbCommandGroup();
    elevatorControl = new ElevatorControl();
    switchControl = new SwitcherControl();
    cargoOutCommand = new CargoOutCommand(0.6,0.6);
    cargoOutLeftCommand = new CargoOutCommand(-1.0,0);
    cargoOutRightCommand = new CargoOutCommand(0,-1.0);
    cargoInCommand = new CargoInCommand();
    

    InitChooser();

    //oi.grabberExtendButton.whenPressed(extenderCommand);
    //oi.flowerButtonL.whenPressed(flowerCommand);
    //oi.flowerButtonR.whenPressed(flowerCommand);
    //oi.reverseButton.whenPressed(reverseCommand);
    oi.encoderResetButton.whenPressed(encoderResetCommand);
    //oi.switcherUpButton.whenPressed(switcherUpCommand);
    //oi.switcherDownButton.whenPressed(switcherDownCommand);
    oi.gameToolIncrementButton.whenPressed(gameToolIncrementCommand);
    oi.gameToolDecrementButton.whenPressed(gameToolDecrementCommand);
    oi.gameToolSwapButton.whenPressed(gameToolSwapCommand);
    oi.gameToolFlowerButton.whenPressed(gameToolFlowerCommand);
    oi.climbButton.whenPressed(climbCommandGroup);
    oi.climbCancelButton.whenPressed(climbCancelCommand);
    oi.cargoInButton.whileHeld(cargoInCommand);
    oi.cargoOutButton.toggleWhenPressed(cargoOutCommand);
    oi.cargoOutLeftButton.toggleWhenPressed(cargoOutLeftCommand);
    oi.cargoOutRightButton.toggleWhenPressed(cargoOutRightCommand);
    
  }

public void InitChooser() {
  m_chooser.setDefaultOption("Center Left to Cargo Left", new PathCommand("CenterLeftToCargoLeft", false));
  m_chooser.addOption("Left Cargo To Platform. (CAUTION)", new PathCommand("LeftCargoToPlatform", false));


  Map<String, Command> pathComands = AutonomousLibrary.GetPathCommands();
  for (Map.Entry<String, Command> entry : pathComands.entrySet()) {
    String pathName = entry.getKey();
    Command pathCommand = entry.getValue();
    m_chooser.addOption(pathName, pathCommand);
  }


   SmartDashboard.putData("Auto mode", m_chooser);
}



  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Elevator Height", elevator.currentHeight());
    SmartDashboard.putNumber("Switcher Height", switcher.getCurrentSwitch());
    SmartDashboard.putNumber("Intake Left Current", cargo.getLeftCurrent());
    SmartDashboard.putNumber("Intake Right Current", cargo.getRightCurrent());

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    if (driveCommand != null && driveCommand.isRunning())
		{
			driveCommand.cancel();
		}
    if(autoHappened){
      driveSubsystem.endPath();
      
      autoHappened = false;
    }
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
boolean autoHappened = false;

  @Override
  public void autonomousInit() {

    //-------EXPERIMENTAL PATH WEAVER CODE-----------
    //switcherEncoderResetCommand.start();

  //  driveSubsystem.PathAutoInit();
    //---------------------------------------------
    
    m_autonomousCommand = m_chooser.getSelected();
    //gameToolStateMachine.autonomousReset();
    

    //drive.motionMagic("aaa");

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
      }
    //driveSubsystem.bullyOff();
    autoHappened = true;
    elevator.encoderStart();
    elevatorControl.start();
    switchControl.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //driveSubsystem.bullyOff();
  }

  boolean boolistatus;
  @Override
  public void teleopInit() {
    
    driveSubsystem.setReverse(false);
    gameToolStateMachine.reset();

		if (driveCommand != null) {
			driveCommand.start();
		}
    
    //-------EXPERIMENTAL PATH WEAVER CODE-----------
    // if(autoHappened){
    //   driveSubsystem.endPath();
    //   autoHappened = false;
    // }
    //--------------------------------------------

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    driveSubsystem.bullyOff();
    boolistatus = false;
    if(!autoHappened){
      elevator.encoderStart();
    }
    elevatorControl.start();
    switchControl.start();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    //System.out.println("Right: " + driveSubsystem.FLMotor.getSelectedSensorPosition(0));
    

    if(oi.leftJoy.getRawButton(3) && boolistatus == false){
      driveSubsystem.switchDriveMode();
      boolistatus = true;
    }else if(!oi.leftJoy.getRawButton(3) && boolistatus == true){
      boolistatus = false;
    }



    //SPARK.set(-0.1);
    //System.out.println("Left Position: " + driveSubsystem.FLMotor.getSelectedSensorPosition(0));
    //System.out.println("Right Position: " + driveSubsystem.FRMotor.getSelectedSensorPosition(0));

    //System.out.println("Front Right Motor Current: " + driveSubsystem.FRMotor.getOutputCurrent());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
