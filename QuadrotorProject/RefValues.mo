within QuadrotorProject;

model RefValues "Generate reference values"
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  RealOutput ref[nRef] "Connector of Real reference signals as output" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
equation
  //  der(ref[1]) = 1 - ref[1]; //refrence signal are the thrust from each motor

/*  ref[1] = 7e-2;
  ref[2] = 7e-2;
  ref[3] = 7e-2;
  ref[4] = 7e-2;
*/
  ref[1] = -1; // reference z
  ref[2] = 0; // z_v
  ref[3] = 0; // roll
  ref[4] = 0; // rollDot
  ref[5] = 0; // pitch
  ref[6] = 0; //pitchDot
  ref[7] = 45; //yaw
  ref[8] = 0; //yawDot
  annotation(Documentation(info = "<html><h1 class=\"heading\">REFERENCE VALUE GENERATION</h1><p>Use this block to generate the reference values for the system.</p></html>"));
end RefValues;