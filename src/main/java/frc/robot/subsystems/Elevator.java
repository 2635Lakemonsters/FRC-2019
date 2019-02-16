package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommand;
//import frc.robot.commands.EmptyCommand;
import frc.robot.subsystems.Elevator.Height;
import frc.robot.subsystems.Switcher.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	WPI_TalonSRX smallMotor;
	WPI_TalonSRX largeMotor1; 
	
	// begin - replacement of 2 bag motors with one full CIM motor
	//WPI_TalonSRX largeMotor2; 
	// end - replacement of 2 bag motors with one full CIM motor
	
	public DigitalInput topLimitSwitch; 
	public DigitalInput bottomLimitSwitch; 
	
	//Arbitrary/Random encoder values for height of locations
	//Ground:0
	//Switch:2000
	//Scale:5000
	//Climb:7000
	
	//Arbitrary/Random encoder values for max elevator heights
	//Lower: 4000
	//Upper: 3000
	
	public Height currentTargetHeight;
	//double largeElevatorMax;
	//double smallElevatorMax;
	
	
	public Elevator() {
		smallMotor  = new WPI_TalonSRX(RobotMap.ELEVATOR_UPPER_MOTOR_CHANNEL);
    	largeMotor1 = new WPI_TalonSRX(RobotMap.ELEVATOR_LOWER_MOTOR1_CHANNEL);

    	
    	//bottomLimitSwitch = new DigitalInput(RobotMap.ELEVATOR_BOTTOM_LIMIT_SWITCH_IO_CHANNEL); 
    	//topLimitSwitch = new DigitalInput(RobotMap.ELEVATOR_TOP_LIMIT_SWITCH_IO_CHANNEL); 

    	// begin - replacement of 2 bag motors with one full CIM motor
    	//largeMotor2 = new WPI_TalonSRX(RobotMap.ELEVATOR_LOWER_MOTOR2_CHANNEL);
    	//largeMotor2.follow(largeMotor1);
    	// end - replacement of 2 bag motors with one full CIM motor
    	
    	encoderStart();
    	//TODO FAKE VALUES, UPDATE TO ACCURATE VALUES, Fix this...
    	// In inches (units)
    	
    	//largeElevatorMax = 17500;
    	//smallElevatorMax = 20000;
    	//END FAKE VALUES
    	double theCurrentHeight = currentHeight();
    	
    	
    	largeMotor1.setSensorPhase(true);
    	
    	// begin - reverse for replacement of bag motor by mini-CIM
    	smallMotor.setInverted(true);
    	smallMotor.setSensorPhase(true);
		// end - reverse for replacement of bag motor by mini-CIM
		
    	currentTargetHeight = Height.GROUND;
    	if (isWithinTolerance(Height.GROUND)) {
    		System.out.println("Initializing target Height to Ground.");
    	}
    	else {
    		System.out.println("WARNING: Current height NOT near ground!!!!. Current Height:" + theCurrentHeight);
    		
    	}
    	
	}
	
	public void motorControl() {
	//System.out.println(currentTargetHeight.toString());
		double lowerHeight = 0;
		double upperHeight = 0;
		
		if(currentTargetHeight.height > RobotMap.SMALL_ELEVATOR_MAX) {
			lowerHeight = RobotMap.SMALL_ELEVATOR_MAX;
			upperHeight = currentTargetHeight.height - RobotMap.SMALL_ELEVATOR_MAX;
			//System.out.println("currentTargetHeight.height > smallElevatorMax");
			
			//System.out.println("Upper height: "+ upperHeight);
			//System.out.println("Lower height: "+ lowerHeight);
		} else {
			upperHeight = 0;
			lowerHeight = currentTargetHeight.height;
			//System.out.println("currentTargetHeight.height < smallElevatorMax");
			
			//System.out.println("Upper height: "+ upperHeight);
			//System.out.println("Lower height: "+ lowerHeight);
		}

		if (currentTargetHeight == Height.GROUND && isWithinTolerance(Height.GROUND)) {
			largeMotor1.set(ControlMode.PercentOutput, 0);
			smallMotor.set(ControlMode.PercentOutput, 0);
			
			
		}
		else {
			largeMotor1.set(ControlMode.MotionMagic, upperHeight);
			smallMotor.set(ControlMode.MotionMagic, lowerHeight);
			
		}
		

	}
	
	public void limitSwitchEncoderReset() {
		//boolean upperLimitSwitchPressed = isLimitSwitchPressed(topLimitSwitch);
		//boolean lowerLimitSwitchPressed = isLimitSwitchPressed(topLimitSwitch);
		
//		if(upperLimitSwitchPressed) {
//			upperMotor.setSelectedSensorPosition(0, 0, 0);
//		}
//		
//		if(lowerLimitSwitchPressed) {
//			lowerMotor1.setSelectedSensorPosition(0, 0, 0);
//		}
	}
	public boolean setTargetHeight(Height height) {
		
		currentTargetHeight = height;
		System.out.println("Setting Target Height: " + height);
		return true;
	}
	
	
	public double currentHeight() {
		
		double upperHeight = Math.abs(largeMotor1.getSelectedSensorPosition(0));
		double lowerHeight = Math.abs(smallMotor.getSelectedSensorPosition(0));
		
		double totalHeight = lowerHeight+upperHeight;
		//System.out.println("lowerHeight:" + lowerHeight + "\tupperHeight:" + upperHeight + "\t totalHeight:" + totalHeight);
		return totalHeight;
	}
	public boolean isLimitSwitchPressed(DigitalInput limitSwitch) {
		boolean result = false;
		try {
			result = !limitSwitch.get();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			result = false;
		}
		return result;
	}
	
	public boolean isFinished(){
		if (isLimitSwitchPressed(bottomLimitSwitch)) {
    		return true;
    	}
		
		if (isLimitSwitchPressed(topLimitSwitch)) {
    		return true;
    	}
		
    	
    	double upperEncoderPosition = smallMotor.getSelectedSensorPosition(0);
    	double lowerEncoderPosition = largeMotor1.getSelectedSensorPosition(1);
		return false;
	}
	
	public boolean isWithinTolerance(Height height) {
		double cHeight = currentHeight();
		
		int lowerError = (height.height - RobotMap.ELEVATOR_TOLERANCE);
		int upperError = (height.height + RobotMap.ELEVATOR_TOLERANCE);
		
		//System.out.println("lowerError: " + lowerError + " upperError: " + upperError);
		
		boolean isOver = (cHeight > lowerError); 
		boolean isUnder = (cHeight < upperError); 
		
		//System.out.println("isOver: " + isOver + " isUnder: " + isUnder);
		
		return isOver && isUnder;
	}
	
	public void encoderStart() {
		
		System.out.println("encoderStart()");
		smallMotor.setSelectedSensorPosition(0, 0, 0);
    	smallMotor.config_kP(0, 5, 0);
    	smallMotor.config_kI(0, 0, 0);
    	smallMotor.config_kD(0, 0, 0);
    	smallMotor.config_kF(0, 0, 0);
    	
    	largeMotor1.setSelectedSensorPosition(0, 0, 0);
    	largeMotor1.config_kP(0, 5, 0);
    	largeMotor1.config_kI(0, 0, 0);
    	largeMotor1.config_kD(0, 0, 0);
    	largeMotor1.config_kF(0, 0, 0);
    	
    	
    	largeMotor1.selectProfileSlot(0, 0);
    	smallMotor.selectProfileSlot(0, 0);
    	
    	largeMotor1.configMotionAcceleration(RobotMap.ELEVATOR_ACCELERATION, 0);
    	smallMotor.configMotionAcceleration(RobotMap.ELEVATOR_ACCELERATION, 0);
    	
    	largeMotor1.configMotionCruiseVelocity(RobotMap.ELEVATOR_VELOCITY, 0);
    	smallMotor.configMotionCruiseVelocity(RobotMap.ELEVATOR_VELOCITY, 0);
	}
	// No longer has fake values <3, this just sets the values for the ground, switch, scale, and climb heights that the elevator uses.
	public static enum Height {
		//GROUND(RobotMap.ELEVATOR_GROUND_LOWER_HEIGHT),
		//EXCHANGE(RobotMap.ELEVATOR_EXCHANGE_LOWER_HEIGHT),
		//STACK(RobotMap.ELEVATOR_STACK_LOWER_HEIGHT),
		//SWITCH(RobotMap.ELEVATOR_SWITCH_LOWER_HEIGHT), 
		//SCALE(RobotMap.ELEVATOR_SCALE_LOWER_HEIGHT), 
		//CLIMB(RobotMap.ELEVATOR_CLIMB_LOWER_HEIGHT);

		GROUND(RobotMap.ELEVATOR_GROUND_HEIGHT),
		LEVEL1H(RobotMap.ELEVATOR_LEVEL1_HATCH_HEIGHT),
		LEVEL1B(RobotMap.ELEVATOR_LEVEL1_BALL_HEIGHT),
		LEVEL2H(RobotMap.ELEVATOR_LEVEL2_HATCH_HEIGHT),
		LEVEL2B(RobotMap.ELEVATOR_LEVEL2_BALL_HEIGHT),
		LEVEL3H(RobotMap.ELEVATOR_LEVEL3_HATCH_HEIGHT),
		LEVEL3B(RobotMap.ELEVATOR_LEVEL3_BALL_HEIGHT);
		
		public int height;
		private Height(int height) {
			this.height = height;
		}
	}

	public Height getUpperHeight(){
		switch(currentTargetHeight){
			case GROUND:
				if(Robot.switcher.currentPosition == Position.HATCH){
					return Height.LEVEL1H;
				}else {
					return Height.LEVEL1B;
				}
			case LEVEL1B:
				return Height.LEVEL2B;
			case LEVEL1H:
				return Height.LEVEL2H;
			case LEVEL2B:
				return Height.LEVEL3B;
			case LEVEL2H:
				return Height.LEVEL3H;
			case LEVEL3B:
				return Height.LEVEL3B;
			case LEVEL3H:
				return Height.LEVEL3H;
			default:
				return Height.GROUND;
		}
	}

	public Height getLowerHeight(){
		switch(currentTargetHeight){
			case GROUND:
				return Height.GROUND;
			case LEVEL1B:
				return Height.GROUND;
			case LEVEL1H:
				return Height.GROUND;
			case LEVEL2B:
				return Height.LEVEL1B;
			case LEVEL2H:
				return Height.LEVEL1H;
			case LEVEL3B:
				return Height.LEVEL2B;
			case LEVEL3H:
				return Height.LEVEL2H;
			default:
				return Height.GROUND;
		}
	}

	public Height getSwappedState(){
		switch(currentTargetHeight){
			case LEVEL1B:
				return Height.LEVEL1H;
			case LEVEL1H:
				return Height.LEVEL1B;
			case LEVEL2B:
				return Height.LEVEL2H;
			case LEVEL2H:
				return Height.LEVEL2B;
			case LEVEL3B:
				return Height.LEVEL3H;
			case LEVEL3H:
				return Height.LEVEL3B;
			default:
				return Height.GROUND;
		}
	}
	
	//Not Used
//	public enum Height2 {
//		//FHE TODO:  
//		GROUND (RobotMap.ELEVATOR_GROUND_LOWER_HEIGHT, RobotMap.ELEVATOR_GROUND_UPPER_HEIGHT),
//		EXCHANGE (RobotMap.ELEVATOR_EXCHANGE_LOWER_HEIGHT, RobotMap.ELEVATOR_EXCHANGE_UPPER_HEIGHT),
//		SWITCH   (RobotMap.ELEVATOR_SWITCH_LOWER_HEIGHT, RobotMap.ELEVATOR_SWITCH_UPPER_HEIGHT),
//		SCALE   (RobotMap.ELEVATOR_SCALE_LOWER_HEIGHT, RobotMap.ELEVATOR_SCALE_UPPER_HEIGHT),
//		CLIMB    (RobotMap.ELEVATOR_CLIMB_LOWER_HEIGHT, RobotMap.ELEVATOR_CLIMB_UPPER_HEIGHT);
//
//
//	    private final double lowerHeight;
//	    private final double upperHeight;
//	    Height2(double lowerHeight, double upperHeight) {
//	        this.lowerHeight = lowerHeight;
//	        this.upperHeight = upperHeight;
//	    }
//	    private double lowerHeight() { return lowerHeight; }
//	    private double upperHeight() { return upperHeight; }
//	}
	
	
//	public  Command ElevatorMove(Height height) {
//
//		 return new ElevatorCommand(height);
//	}
	//pointless ^^^
	
		
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }
    
}

