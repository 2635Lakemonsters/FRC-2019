/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static int LEFT_JOYSTICK_CHANNEL = 0;
  public static int RIGHT_JOYSTICK_CHANNEL = 1;

  public static int FRONT_RIGHT_MOTOR_CHANNEL = 1;
  public static int BACK_RIGHT_MOTOR_CHANNEL = 2;
  public static int FRONT_LEFT_MOTOR_CHANNEL = 3;
  public static int BACK_LEFT_MOTOR_CHANNEL = 4;
  public static int SWITCH_MOTOR_CHANNEL = 7;

  public static int S_PATH_LEFT_BUTTON = 1;

  public static int SWITCHER_VELOCITY = 10;

  //--------EXPERIMENTAL PATHWEAVER CONSTANTS------------
  public static final int k_ticks_per_rev = 4096/3;
  public static final double k_wheel_diameter = 0.49; //feet!
  public static final double k_max_velocity = 10.0;

  // public static final int k_left_channel = 0;
  // public static final int k_right_channel = 1;


  // public static final int k_left_encoder_port_a = 0;
  // public static final int k_left_encoder_port_b = 1;
  // public static final int k_right_encoder_port_a = 2;
  // public static final int k_right_encoder_port_b = 3;

  public static final int k_gyro_port = 0;

  /* Elevator Values */
  public static final int ELEVATOR_TOLERANCE = 2000;
  public static final int ELEVATOR_UPPER_MOTOR_CHANNEL = 0;
  public static final int ELEVATOR_LOWER_MOTOR1_CHANNEL = 0;
  public static final int SMALL_ELEVATOR_MAX = 20000;
  public static final int ELEVATOR_ACCELERATION = 5000;
  public static final int ELEVATOR_VELOCITY = 3000;
  public static final int ELEVATOR_GROUND_LOWER_HEIGHT = 0;
  public static final int ELEVATOR_EXCHANGE_LOWER_HEIGHT = 0;
  public static final int ELEVATOR_STACK_LOWER_HEIGHT = 0;
  public static final int ELEVATOR_SWITCH_LOWER_HEIGHT = 0;
  public static final int ELEVATOR_SCALE_LOWER_HEIGHT = 0;
  public static final int ELEVATOR_CLIMB_LOWER_HEIGHT = 0;

  //public static final String k_path_name = "example";
  //------------------------------------------------------
  
}
