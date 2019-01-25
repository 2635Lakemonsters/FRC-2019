# FRC-2019

Table of Contents

[Pathweaver](#pathweaver)

[Pathweaver Code](#pathcode)

[Using the Wrapper](#usewrapper)

[Generating Trajectory](#gentraj)

[Use Trajectory](#usetraj)

[Tank Drive](#tank)

[Swerve Drive](#swerve)

[Controls](#control)

<a name="pathweaver"></a>
# Pathweaver
All pathweaver paths are in the src/main/deploy.
Look in PathDatum for initalizing these paths.
Implemented in DriveSubsystem and PathTestCommand.
See Integrating path following into a robot program for pattern to implement path into a robot:

https://wpilib.screenstepslive.com/s/currentCS/m/84338/l/1021631-integrating-path-following-into-a-robot-program


<a name="pathcode"></a>
## Pathweaver Code
Jaci Packages

```java
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
```

Pathfinder 2.0 Documentation and Java API:

https://github.com/JacisNonsense/Pathfinder

https://github.com/JacisNonsense/Pathfinder/tree/master/Pathfinder-Java

<a name="usewrapper"></a>
### Using the Wrapper (Taken From API):

<a name="gentraj"></a>
Generating Trajectory

```java
// Create 3 Waypoints

Waypoint[] points = new Waypoint[] {
    new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
    new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
    new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
};

// Trajectory Configuration
//
// Arguments:
// Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
// Sample Count:        SAMPLES_HIGH (100 000)
//                      SAMPLES_LOW  (10 000)
//                      SAMPLES_FAST (1 000)
// Time Step:           0.05 Seconds
// Max Velocity:        1.7 m/s
// Max Acceleration:    2.0 m/s/s
// Max Jerk:            60.0 m/s/s/s
Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);

// Generate Trajectory
Trajectory trajectory = Pathfinder.generate(points, config);
```

<a name="usetraj"></a>
Using Trajectory

```java
for (int i = 0; i < trajectory.length(); i++) {
    Trajectory.Segment seg = trajectory.get(i);
    
    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
        seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
            seg.acceleration, seg.jerk, seg.heading);
}
```
Modify Trajectory for different drivetrains

<a name="tank"></a>
Tank Drive
```java
// The distance between the left and right sides of the wheelbase is 0.6m
double wheelbase_width = 0.6;

// Create the Modifier Object
TankModifier modifier = new TankModifier(trajectory);

// Generate the Left and Right trajectories using the original trajectory
// as the centre
modifier.modify(wheelbase_width);

Trajectory left  = modifier.getLeftTrajectory();       // Get the Left Side
Trajectory right = modifier.getRightTrajectory();      // Get the Right Side
```

<a name="swerve"></a>
Swerve Drive
```java
// The distance between the left and right sides of the wheelbase is 0.6m
double wheelbase_width = 0.6;

// The distance between the front and back sides of the wheelbase is 0.5m
double wheelbase_depth = 0.5;

// The swerve mode to generate will be the 'default' mode, where the 
// robot will constantly be facing forward and 'sliding' sideways to 
// follow a curved path.
SwerveModifier.Mode mode = SwerveModifier.Mode.SWERVE_DEFAULT;

// Create the Modifier Object
SwerveModifier modifier = new SwerveModifier(trajectory);

// Generate the individual wheel trajectories using the original trajectory
// as the centre
modifier.modify(wheelbase_width, wheelbase_depth, mode);

Trajectory fl = modifier.getFrontLeftTrajectory();       // Get the Front Left wheel
Trajectory fr = modifier.getFrontRightTrajectory();      // Get the Front Right wheel
Trajectory bl = modifier.getBackLeftTrajectory();        // Get the Back Left wheel
Trajectory br = modifier.getBackRightTrajectory();       // Get the Back Right wheel
```

<a name="control"></a>
# Controls
Control implementation with Joysticks has not been Finsished.
Button Mapping:
1. LJoy1 - DriveSwitch
2. RJoy1 - Kicker

The Interface Control Document (ICD) can be found here
https://docs.google.com/spreadsheets/d/1G7-52sN1RaOKleUrd1jPt_BSWh4CiMyCfBkSfKL26AA/edit?usp=sharing

Written with love in Virtual Studio.
