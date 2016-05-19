within QuadrotorProject;

model Controller
  extends Modelica.Blocks.Icons.Block;
  //Import libraries and config
  import QuadrotorProject.Config.*;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.SIunits;
  RealInput ref[nRef] "Connector of Real reference signals as input" annotation(Placement(transformation(extent = {{-140, 20}, {-100, 60}}, rotation = 0)));
  RealInput sens[nSens] "Connector of Real sensor signals as input" annotation(Placement(transformation(extent = {{-140, -60}, {-100, -20}}, rotation = 0)));
  RealOutput ctrl[nCtrl] "Connector of Real control signals as output" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add[nCtrl](k1 = 1, k2 = -1) annotation(Placement(visible = true, transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Math.MatrixGain K(K = [
   -Ka, -Kb, Ka,   Kb,   Kc,  Kc;
   -Ka, -Kb, -Ka, -Kb,  -Kc, -Kc;
   Ka,  Kb,  -Ka,  -Kb,   Kc,  Kc;
   Ka,  Kb,  Ka,   Kb,  -Kc, -Kc])            
                                          annotation(Placement(visible = true, transformation(origin = {-64, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                                          
equation
  connect(ref, add.u1) annotation(Line(points = {{-120, 40}, {-40, 40}, {-40, 6}, {-10, 6}, {-10, 6}}, color = {0, 0, 127}));
  connect(K.y, add.u2) annotation(Line(points = {{-52, -40}, {-20, -40}, {-20, -6}, {-10, -6}, {-10, -6}}, color = {0, 0, 127}));
  connect(K.u, sens) annotation(Line(points = {{-76, -40}, {-102, -40}, {-102, -40}, {-120, -40}}, color = {0, 0, 127}));
  

  for i in 1:size(add,1) loop
              // motors limit thurst to around .6/4 Newtons
    ctrl[i] = if (add[i].y>=0.15) then 0.15
              // motors cannot yield force downwards
              elseif (add[i].y<=0) then 0
              else add[i].y;
  end for;
  annotation(Documentation(info = "<html><h1 class=\"heading\">CONTROLLER BLOCK WITH BUILT IN MODELICA PID</h1><p>Uses only default modelica blocks to model the controller</p></html>"), uses(Modelica(version = "3.2.1")));
end Controller;