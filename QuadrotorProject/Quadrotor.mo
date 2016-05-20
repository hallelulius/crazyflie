within QuadrotorProject;

model Quadrotor
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  // Inputs should be all controll signals from your controller
  RealInput ctrl[nCtrl] "Connector of Real control signals as input" annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
  // Output should be all states and output that you want to be able to use in the Sensor block.
  RealOutput plantOut[nOut] "Connector of Real plant outputs as output" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  // Quadrotor components
  Real T, z, y, x, z_v, y_v, x_v, rollDot, pitchDot, yawDot, pitch, roll, yaw;
initial equation
x = 0;
y = 0;
z = 0;
pitch = 1.2;
roll = -0.9;
yaw = -0.3;
equation
  //Nonlinear model of a x-configuration quadrotor
  
  // Total motor thrust
  T = ctrl[1] + ctrl[2] + ctrl[3] + ctrl[4];

  // translation motion
  der(x) = x_v;
  m*der(x_v) = T * (sin(roll)* sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)); 
  
  der(y) = y_v;
  m*der(y_v) = T * (cos(roll)* sin(pitch) *sin(yaw) - cos(yaw) * sin(roll));  
  
  der(z) = z_v;
  m*der(z_v) = m*g -cos(roll) * cos(pitch) * T; 
  
  
  //rotational movemen
  der(roll) = rollDot;   // around x axis
  Ix * der(rollDot) = (sqrt(2)/2) * L* (-ctrl[1] -ctrl[2] + ctrl[3] + ctrl[4]) +
                            (Iy - Iz) * pitchDot * yawDot;  
  der(pitch) = pitchDot; // around y-axis
  Iy *der(pitchDot) = (sqrt(2)/2)* L* (ctrl[1] - ctrl[2] - ctrl[3] + ctrl[4]) +
                         (Iz - Ix) * rollDot * yawDot;  
  der(yaw) = yawDot;    // around z-azis
  Iz*der(yawDot) = (ctrl[1] - ctrl[2] + ctrl[3] - ctrl[4]) * L * k/b -
                      (Ix - Iy) * pitchDot * rollDot;
  
  //plant output                    
  z = plantOut[1];
  z_v = plantOut[2];
  y = plantOut[3];
  y_v = plantOut[4];
  x = plantOut[5];
  x_v = plantOut[6]; 
    
  roll = plantOut[7];
  rollDot = plantOut[8];
  pitch = plantOut[9];
  pitchDot = plantOut[10];
  yaw = plantOut[11];
  yawDot = plantOut[12];

  // NOTHING MORE TO CHANGE BELOW
  annotation(Documentation(info = "<html><h1 class=\"heading\">QUADROTOR BLOCK</h1><p></p><p>Implement your own plant model equation and update the force and torque that affects the Body of the quadrotor.</p></html>"));
end Quadrotor;