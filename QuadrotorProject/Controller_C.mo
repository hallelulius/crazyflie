within QuadrotorProject;

model Controller_C
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  RealInput ref[nRef] "Connector of Real reference signals as input" annotation(Placement(transformation(extent = {{-140, 20}, {-100, 60}}, rotation = 0)));
  RealInput sens[nSens] "Connector of Real sensor signals as input" annotation(Placement(transformation(extent = {{-140, -60}, {-100, -20}}, rotation = 0)));
  RealOutput ctrl[nSens] "Connector of Real control signals as output" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add[nSens](k2 = -1) annotation(Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  impure function pid "Modelica wrapper for an embedded C controller"
    input Real e;
    output Real u;
  
    external "C"  annotation(Include = "#include \"pid.c\"", IncludeDirectory = "modelica://QuadrotorProject/source");
  end pid;

  // doesnt work
equation
  connect(ref[:], add[:].u1) annotation(Line(points = {{-120, 40}, {-80, 40}, {-80, 6}, {-62, 6}}, color = {0, 0, 127}));
  connect(sens[:], add[:].u2) annotation(Line(points = {{-120, -40}, {-80, -40}, {-80, -6}, {-62, -6}}, color = {0, 0, 127}));
  when sample(0, 0.001) then
    ctrl = add.y;
  end when;
  annotation(Documentation(info = "<html><h1 class=\"heading\">CONTROLLER BLOCK WITH BUILT SOFTWARE IN LOOP CONTROLLER</h1><p>Uses C-code to implement PID controller</p></html>"));
end Controller_C;