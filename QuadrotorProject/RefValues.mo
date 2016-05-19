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

  ref[1] = m*g/4;
  ref[2] = m*g/4;
  ref[3] = m*g/4;
  ref[4] = m*g/4;

  annotation(Documentation(info = "<html><h1 class=\"heading\">REFERENCE VALUE GENERATION</h1><p>Use this block to generate the reference values for the system.</p></html>"));
end RefValues;