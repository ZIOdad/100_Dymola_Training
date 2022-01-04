within ;
package DymolaIntroductionCourse
  package Furuta_Pendulum
    model Furuta "Furuta pendulum with 3D animation"

      inner Modelica.Mechanics.MultiBody.World world
                                  annotation (Placement(transformation(extent={{-80,
                0},{-60,20}}, rotation=0)));
      Modelica.Mechanics.MultiBody.Joints.Revolute R1(
        n={0,1,0},
        a(fixed=false),
        phi(fixed=true, start=0.26179938779915),
        w(fixed=true))
        annotation (Placement(transformation(
            origin={-50,50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Mechanics.MultiBody.Parts.BodyBox B1(color={255,0,0}, r={0.5,0,0})
        annotation (Placement(transformation(extent={{-30,60},{-10,80}}, rotation=0)));
      Modelica.Mechanics.MultiBody.Joints.Revolute R2(
        n={1,0,0},
        a(fixed=false),
        w(fixed=true),
        phi(
          fixed=true,
          start=2,
          displayUnit="rad")) annotation (Placement(transformation(extent={{0,60},{
                20,80}}, rotation=0)));
      Modelica.Mechanics.MultiBody.Parts.BodyBox B2(color={0,180,0}, r={0,-0.5,0})
        annotation (Placement(transformation(
            origin={30,30},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Mechanics.MultiBody.Joints.Revolute R3(
        n={1,0,0},
        a(fixed=false),
        phi(fixed=true),
        w(fixed=true))        annotation (Placement(transformation(extent={{40,-20},
                {60,0}}, rotation=0)));
      Modelica.Mechanics.MultiBody.Parts.BodyBox B3(color={0,0,255}, r={0,-0.5,0})
        annotation (Placement(transformation(
            origin={70,-50},
            extent={{-10,-10},{10,10}},
            rotation=270)));
    equation
      connect(R3.frame_b, B3.frame_a) annotation (Line(
          points={{60,-10},{70,-10},{70,-40}},
          color={0,0,0},
          thickness=0.5));
      connect(world.frame_b, R1.frame_a) annotation (Line(
          points={{-60,10},{-50,10},{-50,40}},
          color={0,0,0},
          thickness=0.5));
      connect(R1.frame_b, B1.frame_a) annotation (Line(
          points={{-50,60},{-50,70},{-30,70}},
          color={0,0,0},
          thickness=0.5));
      connect(B1.frame_b, R2.frame_a) annotation (Line(
          points={{-10,70},{0,70}},
          color={0,0,0},
          thickness=0.5));
      connect(R2.frame_b, B2.frame_a) annotation (Line(
          points={{20,70},{30,70},{30,40}},
          color={0,0,0},
          thickness=0.5));
      connect(B2.frame_b, R3.frame_a) annotation (Line(
          points={{30,20},{30,-10},{40,-10}},
          color={0,0,0},
          thickness=0.5));
      annotation (
     versionBuild=1,
        versionDate="2019-02-27",
     dateModified="2019-02-27 15:30:00Z",
        __Dymola_Commands(file="Furuta.mos" "Simulate Furuta pendulum", file="Animate.mos"
            "Animate Furuta pendulum"),
        experiment(StopTime=25));
    end Furuta;
  end Furuta_Pendulum;

  package Electric_Motor
    model ElectricMotorwithControl
      Modelica.Blocks.Sources.Constant const(k=1)
        annotation (Placement(transformation(extent={{-40,80},{-20,100}})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=
            100) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,50})));
      Modelica.Thermal.HeatTransfer.Components.Convection convection
        annotation (Placement(transformation(extent={{40,40},{60,60}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
            displayUnit="K") = 293)
        annotation (Placement(transformation(extent={{140,40},{120,60}})));
      Modelica.Blocks.Sources.TimeTable timeTable(table=[0,0; 1,0; 2,1; 3,1; 4,
            0; 5,0; 6,-1; 7,-1; 8,0; 9,0])
        annotation (Placement(transformation(extent={{-100,-20},{-80,0}})));
      Modelica.Blocks.Continuous.LimPID PID(
        k=5,
        Ti=0.1,
        yMax=42)
        annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-8,-10})));
      Modelica.Electrical.Analog.Basic.HeatingResistor resistor
        annotation (Placement(transformation(extent={{4,30},{24,10}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
      Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet
        dcpm(
        VaNominal=42,
        IaNominal=50,
        wNominal(displayUnit="rpm") = 26.179938779915)
        annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(useSupport=
            true, ratio=5)
        annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.2, w(fixed=
              true, start=1))
        annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={140,-70})));
    equation
      connect(heatCapacitor.port, convection.solid)
        annotation (Line(points={{-40,50},{40,50}}, color={191,0,0}));
      connect(convection.fluid, fixedTemperature.port)
        annotation (Line(points={{60,50},{120,50}}, color={191,0,0}));
      connect(idealGear.flange_a, dcpm.flange)
        annotation (Line(points={{60,-50},{40,-50}}, color={0,0,0}));
      connect(const.y, convection.Gc)
        annotation (Line(points={{-19,90},{50,90},{50,60}}, color={0,0,127}));
      connect(resistor.heatPort, convection.solid)
        annotation (Line(points={{14,30},{14,50},{40,50}}, color={191,0,0}));
      connect(timeTable.y, PID.u_s)
        annotation (Line(points={{-79,-10},{-62,-10}}, color={0,0,127}));
      connect(signalVoltage.v, PID.y)
        annotation (Line(points={{-20,-10},{-39,-10}}, color={0,0,127}));
      connect(signalVoltage.p, resistor.p)
        annotation (Line(points={{-8,0},{-8,20},{4,20}}, color={0,0,255}));
      connect(signalVoltage.n, ground.p) annotation (Line(points={{-8,-20},{-10,
              -20},{-10,-40}}, color={0,0,255}));
      connect(dcpm.pin_an, ground.p)
        annotation (Line(points={{24,-40},{-10,-40}}, color={0,0,255}));
      connect(dcpm.pin_ap, resistor.n)
        annotation (Line(points={{36,-40},{36,20},{24,20}}, color={0,0,255}));
      connect(idealGear.flange_b, inertia.flange_a)
        annotation (Line(points={{80,-50},{100,-50}}, color={0,0,0}));
      connect(inertia.flange_b, speedSensor.flange) annotation (Line(points={{
              120,-50},{140,-50},{140,-60}}, color={0,0,0}));
      connect(speedSensor.w, PID.u_m) annotation (Line(points={{140,-81},{140,
              -102},{-50,-102},{-50,-22}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{
                160,120}})),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},
                {160,120}})),
        experiment(StopTime=9, __Dymola_Algorithm="Dassl"));
    end ElectricMotorwithControl;
  end Electric_Motor;

  package Motor_Drive
    model DC_Motor
      Modelica.Electrical.Analog.Basic.Resistor Ra(R=0.5)
        annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
      Modelica.Electrical.Analog.Basic.Inductor La(L=0.05)
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{30,-10},{50,10}})));
      Modelica.Electrical.Analog.Basic.Ground G
        annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage Vs annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-60,0})));
      Modelica.Mechanics.Rotational.Components.Inertia Jm(J=0.001)
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                        "Flange of right shaft"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealInput v1
        "Voltage between pin p and n (= p.v - n.v) as input signal" annotation (
         Placement(transformation(extent={{-120,-20},{-80,20}}),
            iconTransformation(extent={{-120,-20},{-80,20}})));
    equation
      connect(Ra.n, La.p)
        annotation (Line(points={{-20,30},{0,30}}, color={0,0,255}));
      connect(Vs.p, Ra.p) annotation (Line(points={{-60,10},{-60,30},{-40,30}},
            color={0,0,255}));
      connect(Vs.n, G.p) annotation (Line(points={{-60,-10},{-60,-40},{-50,-40}},
            color={0,0,255}));
      connect(G.p, emf.n) annotation (Line(points={{-50,-40},{40,-40},{40,-10}},
            color={0,0,255}));
      connect(La.n, emf.p)
        annotation (Line(points={{20,30},{40,30},{40,10}}, color={0,0,255}));
      connect(emf.flange, Jm.flange_a)
        annotation (Line(points={{50,0},{60,0}}, color={0,0,0}));
      connect(Jm.flange_b, flange_b1)
        annotation (Line(points={{80,0},{100,0}}, color={0,0,0}));
      connect(Vs.v, v1) annotation (Line(points={{-72,8.88178e-16},{-92,
              8.88178e-16},{-92,0},{-100,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-70,40},{70,-40}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={238,46,47}),
            Polygon(
              points={{-50,-20},{50,-20},{60,-60},{80,-60},{80,-80},{-80,-80},{
                  -80,-60},{-60,-60},{-50,-20}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={0,0,0}),
            Text(
              extent={{-78,80},{80,40}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={0,0,0},
              textString="DC Motor"),
            Rectangle(
              extent={{70,10},{100,-10}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={95,95,95})}), Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end DC_Motor;

    model Test_Motor
      DC_Motor dC_Motor
        annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    equation
      connect(step.y, dC_Motor.v1)
        annotation (Line(points={{-59,0},{-20,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_Motor;

    model Motor_Drive
      DC_Motor dC_Motor
        annotation (Placement(transformation(extent={{-4,-14},{20,14}})));
      Modelica.Blocks.Math.Feedback PositionError
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      Modelica.Blocks.Continuous.PID Controller(
        k=2,
        Ti=1000,
        Td=0.001)
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Modelica.Mechanics.Rotational.Components.IdealGear Gearbox(ratio=3)
        annotation (Placement(transformation(extent={{28,-10},{48,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia Load(J=10)
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor Phiload annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={90,-50})));
    equation
      connect(dC_Motor.v1, Controller.y) annotation (Line(points={{-4,
              1.77636e-15},{-20,1.77636e-15},{-20,0},{-19,0}}, color={0,0,127}));
      connect(PositionError.y, Controller.u)
        annotation (Line(points={{-51,0},{-42,0}}, color={0,0,127}));
      connect(dC_Motor.flange_b1, Gearbox.flange_a) annotation (Line(points={{
              20,1.77636e-15},{28,1.77636e-15},{28,0}}, color={0,0,0}));
      connect(Gearbox.flange_b, Load.flange_a)
        annotation (Line(points={{48,0},{60,0}}, color={0,0,0}));
      connect(Load.flange_b, Phiload.flange)
        annotation (Line(points={{80,0},{90,0},{90,-40}}, color={0,0,0}));
      connect(Phiload.phi, PositionError.u2) annotation (Line(points={{90,-61},
              {90,-80},{-60,-80},{-60,-8}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor_Drive;

    model MotorDrive_Test
      extends Motor_Drive(Controller(
          k=12.0,
          Ti=1000,
          Td=0.4));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    equation
      connect(PositionError.u1, step.y)
        annotation (Line(points={{-68,0},{-79,0}}, color={0,0,127}));
      annotation (experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
    end MotorDrive_Test;
  end Motor_Drive;

  package Double_Pendulum

    model DoublePendulum
      "Simple double pendulum with two revolute joints and two bodies"

      extends Modelica.Icons.Example;
      inner Modelica.Mechanics.MultiBody.World world annotation (Placement(
            transformation(extent={{-100,-10},{-80,10}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(useAxisFlange=true,phi(fixed=true),
          w(fixed=true)) annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Mechanics.Rotational.Components.Damper damper(d=0.3)
        annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
      Modelica.Mechanics.MultiBody.Parts.BodyBox boxBody1(r={0.5,0,0}, width=0.06)
        annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute2(phi(fixed=true), w(
            fixed=true)) annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Modelica.Mechanics.MultiBody.Parts.BodyBox boxBody2(r={0.5,0,0}, width=0.06)
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
    equation

      connect(damper.flange_b, revolute1.axis) annotation (Line(points={{-40,40},{-40,20},{-50,20},{-50,10}}));
      connect(revolute1.support, damper.flange_a) annotation (Line(points={{-56,10},{-56,20},{-60,20},{-60,40}}));
      connect(revolute1.frame_b, boxBody1.frame_a)
        annotation (Line(
          points={{-40,0},{-20,0}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute2.frame_b, boxBody2.frame_a)
        annotation (Line(
          points={{40,0},{60,0}},
          color={95,95,95},
          thickness=0.5));
      connect(boxBody1.frame_b, revolute2.frame_a)
        annotation (Line(
          points={{0,0},{20,0}},
          color={95,95,95},
          thickness=0.5));
      connect(world.frame_b, revolute1.frame_a)
        annotation (Line(
          points={{-80,0},{-60,0}},
          color={95,95,95},
          thickness=0.5));
      annotation (
        experiment(StopTime=10, __Dymola_Algorithm="Dassl"),
        Documentation(info="<html>
<p>
This example demonstrates that by using joint and body
elements animation is automatically available. Also the revolute
joints are animated. Note, that animation of every component
can be switched of by setting the first parameter <strong>animation</strong>
to <strong>false</strong> or by setting <strong>enableAnimation</strong> in the <strong>world</strong>
object to <strong>false</strong> to switch off animation of all components.
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/DoublePendulum.png\"
alt=\"model Examples.Elementary.DoublePendulum\">
</blockquote>
</html>"));
    end DoublePendulum;
  end Double_Pendulum;

  package DC_Motor
    partial model PartialBasicMachine "Partial model for all machines"
      import Modelica.Constants.pi;
      extends Modelica.Electrical.Machines.Icons.TransientMachine;
      parameter Modelica.SIunits.Inertia Jr "Rotor's moment of inertia";
      parameter Boolean useSupport=false
        "Enable / disable (=fixed stator) support" annotation (Evaluate=true);
      parameter Modelica.SIunits.Inertia Js=Jr "Stator's moment of inertia"
                                     annotation (Dialog(enable=useSupport));
      parameter Boolean useThermalPort=false
        "Enable / disable (=fixed temperatures) thermal port"
        annotation (Evaluate=true);
    //  parameter Modelica.Electrical.Machines.Losses.FrictionParameters frictionParameters
    //    "Friction loss parameter record" annotation (Dialog(tab="Losses"));
      output Modelica.SIunits.Angle phiMechanical(start=0) = flange.phi -
        internalSupport.phi "Mechanical angle of rotor against stator";
      output Modelica.SIunits.AngularVelocity wMechanical(
        displayUnit="rev/min",
        start=0) = der(phiMechanical)
        "Mechanical angular velocity of rotor against stator";
      output Modelica.SIunits.Torque tauElectrical=inertiaRotor.flange_a.tau
        "Electromagnetic torque";
      output Modelica.SIunits.Torque tauShaft=-flange.tau "Shaft torque";
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange "Shaft"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertiaRotor(final J=Jr)
        annotation (Placement(transformation(
            origin={80,0},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a support if useSupport
        "Support at which the reaction torque is acting" annotation (Placement(
            transformation(extent={{90,-110},{110,-90}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertiaStator(final J=Js)
        annotation (Placement(transformation(
            origin={80,-100},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Mechanics.Rotational.Components.Fixed fixed if (not useSupport)
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=180,
            origin={50,-100})));
    protected
      Modelica.Mechanics.Rotational.Interfaces.Support internalSupport
        annotation (Placement(transformation(extent={{56,-104},{64,-96}})));
    equation
      connect(inertiaRotor.flange_b, flange) annotation (Line(points={{90,0},{
              92,0},{92,0},{100,0}}));
      connect(inertiaStator.flange_b, support)
        annotation (Line(points={{90,-100},{100,-100}}));
      connect(internalSupport, fixed.flange) annotation (Line(
          points={{60,-100},{50,-100}}));
      connect(internalSupport, inertiaStator.flange_a) annotation (Line(
          points={{60,-100},{70,-100}}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={
            Text(
              extent={{-150,-120},{150,-160}},
              lineColor={0,0,255},
              textString="%name"),
            Rectangle(
              extent={{80,-80},{120,-120}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              visible=not useSupport,
              points={{80,-100},{120,-100}}),
            Line(
              visible=not useSupport,
              points={{90,-100},{80,-120}}),
            Line(
              visible=not useSupport,
              points={{100,-100},{90,-120}}),
            Line(
              visible=not useSupport,
              points={{110,-100},{100,-120}}),
            Line(
              visible=not useSupport,
              points={{120,-100},{110,-120}})}), Documentation(info="<html>
Base partial model DC machines:
<ul>
<li>main parts of the icon</li>
<li>mechanical shaft</li>
<li>mechanical support</li>
</ul>
Besides the mechanical connector <em>flange</em> (i.e., the shaft) the machines have a second mechanical connector <em>support</em>.<br>
If <em>useSupport</em> = false, it is assumed that the stator is fixed.<br>
Otherwise reaction torque (i.e., air gap torque, minus acceleration torque for stator's moment of inertia) can be measured at <em>support</em>.<br>
One may also fix the shaft and let rotate the stator; parameter Js is only of importance when the stator is rotating.
</html>"));
    end PartialBasicMachine;

    model DCMotor
      extends PartialBasicMachine;
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.05)
        annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.01)
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Modelica.Electrical.Analog.Basic.EMF emf(k=0.3)
        annotation (Placement(transformation(extent={{24,-10},{44,10}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin p1
                    "Positive electrical pin"
        annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin n1
                               "Negative electrical pin"
        annotation (Placement(transformation(extent={{-100,-30},{-80,-10}})));
    equation
      connect(resistor.p, p1)
        annotation (Line(points={{-60,30},{-90,30}}, color={0,0,255}));
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-40,30},{0,30}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{20,30},{34,30},{34,10}}, color={0,0,255}));
      connect(emf.n, n1) annotation (Line(points={{34,-10},{34,-20},{-90,-20}},
            color={0,0,255}));
      connect(emf.flange, inertiaRotor.flange_a) annotation (Line(points={{44,0},
              {58,0},{58,1.77636e-15},{70,1.77636e-15}}, color={0,0,0}));
    end DCMotor;

    model MotorTest
      DCMotor dCMotor(Jr=0.2)
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=8)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=100)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-70,0})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
    equation
      connect(dCMotor.flange, inertia.flange_a)
        annotation (Line(points={{8,0},{40,0}}, color={0,0,0}));
      connect(constantVoltage.p, dCMotor.p1) annotation (Line(points={{-70,10},
              {-70,38},{-20,38},{-20,3},{-11,3}}, color={0,0,255}));
      connect(constantVoltage.n, ground.p)
        annotation (Line(points={{-70,-10},{-70,-40}}, color={0,0,255}));
      connect(ground.p, dCMotor.n1) annotation (Line(points={{-70,-40},{-20,-40},
              {-20,-2},{-11,-2}}, color={0,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=20, __Dymola_Algorithm="Dassl"));
    end MotorTest;
  end DC_Motor;

  package Inverse_Pendulum
    model Pendulum
      inner Modelica.Mechanics.MultiBody.World world
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r={
            0.25,0.125,0})
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute(phi(fixed=true,
            start=1.6580627893946))
        annotation (Placement(transformation(extent={{40,20},{60,40}})));
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodyCylinder(r={1,0,0})
        annotation (Placement(transformation(extent={{80,20},{100,40}})));
      Modelica.Mechanics.MultiBody.Parts.BodyBox bodyBox(
        r={0.5,0,0},
        length=0.5,
        width=0.25,
        height=0.25)
        annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
    equation
      connect(world.frame_b, prismatic.frame_a) annotation (Line(
          points={{-60,0},{-40,0}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation.frame_b, revolute.frame_a) annotation (Line(
          points={{20,30},{40,30}},
          color={95,95,95},
          thickness=0.5));
      connect(prismatic.frame_b, fixedTranslation.frame_a) annotation (Line(
          points={{-20,0},{-8,0},{-8,30},{0,30}},
          color={95,95,95},
          thickness=0.5));
      connect(bodyCylinder.frame_a, revolute.frame_b) annotation (Line(
          points={{80,30},{60,30}},
          color={95,95,95},
          thickness=0.5));
      connect(bodyBox.frame_a, prismatic.frame_b) annotation (Line(
          points={{40,-30},{-8,-30},{-8,0},{-20,0}},
          color={95,95,95},
          thickness=0.5));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
    end Pendulum;
  end Inverse_Pendulum;

  package Flying_Gull
    model Gull_1
      inner Modelica.Mechanics.MultiBody.World world
        annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange=
            true, n(displayUnit="1") = {0,1,0})
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation_R(r={0.3,0,
            -0.04})
        annotation (Placement(transformation(extent={{-20,40},{0,60}})));
      Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation_L(r={0.3,0,
            0.04})
        annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute_R(n(displayUnit="1")
           = {1,0,0})
        annotation (Placement(transformation(extent={{20,40},{40,60}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute_L(n(displayUnit="1")
           = {1,0,0})
        annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder Gull_Body(
        r={0.6,0,0},
        diameter=0.08,
        density(displayUnit="kg/m3") = 200)
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
      Modelica.Mechanics.MultiBody.Parts.BodyBox Wing_R(r={0,0,-1})
        annotation (Placement(transformation(extent={{60,10},{40,30}})));
      Modelica.Mechanics.MultiBody.Parts.BodyBox Wing_L(
        r={0,0,1},
        width=0.005,
        height=0.12,
        density(displayUnit="kg/m3") = 1000)
        annotation (Placement(transformation(extent={{60,-30},{40,-10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation_R1(r
          ={0.3,1.5,-1/3 - 0.04})
        annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation_L1(r
          ={0.3,1.5,1/3 + 0.04})
        annotation (Placement(transformation(extent={{-60,-90},{-40,-70}})));
      Modelica.Mechanics.MultiBody.Joints.SphericalSpherical
        sphericalSpherical_R(rodLength=1.5)
        annotation (Placement(transformation(extent={{-8,70},{12,90}})));
      Modelica.Mechanics.MultiBody.Joints.SphericalSpherical
        sphericalSpherical_L(rodLength=1.5)
        annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation_L2(r
          ={0,0,-1/3})
        annotation (Placement(transformation(extent={{40,-90},{60,-70}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation_R2(r
          ={0,0,1/3})
        annotation (Placement(transformation(extent={{40,70},{60,90}})));
    equation
      connect(world.frame_b, prismatic.frame_a) annotation (Line(
          points={{-72,0},{-60,0}},
          color={95,95,95},
          thickness=0.5));
      connect(prismatic.frame_b, fixedRotation_R.frame_a) annotation (Line(
          points={{-40,0},{-30,0},{-30,50},{-20,50}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedRotation_L.frame_a, fixedRotation_R.frame_a) annotation (
          Line(
          points={{-20,-50},{-30,-50},{-30,50},{-20,50}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedRotation_R.frame_b, revolute_R.frame_a) annotation (Line(
          points={{8.88178e-16,50},{20,50}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedRotation_L.frame_b, revolute_L.frame_a) annotation (Line(
          points={{8.88178e-16,-50},{20,-50}},
          color={95,95,95},
          thickness=0.5));
      connect(prismatic.frame_b, Gull_Body.frame_a) annotation (Line(
          points={{-40,0},{0,0}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute_R.frame_b, Wing_R.frame_a) annotation (Line(
          points={{40,50},{80,50},{80,20},{60,20}},
          color={95,95,95},
          thickness=0.5));
      connect(revolute_L.frame_b, Wing_L.frame_a) annotation (Line(
          points={{40,-50},{80,-50},{80,-20},{60,-20}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation_R1.frame_a, world.frame_b) annotation (Line(
          points={{-60,80},{-66,80},{-66,0},{-72,0}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation_L1.frame_a, world.frame_b) annotation (Line(
          points={{-60,-80},{-66,-80},{-66,0},{-72,0}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation_R1.frame_b, sphericalSpherical_R.frame_a)
        annotation (Line(
          points={{-40,80},{-8,80}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation_L1.frame_b, sphericalSpherical_L.frame_a)
        annotation (Line(
          points={{-40,-80},{-10,-80}},
          color={95,95,95},
          thickness=0.5));
      connect(sphericalSpherical_R.frame_b, fixedTranslation_R2.frame_a)
        annotation (Line(
          points={{12,80},{40,80}},
          color={95,95,95},
          thickness=0.5));
      connect(sphericalSpherical_L.frame_b, fixedTranslation_L2.frame_a)
        annotation (Line(
          points={{10,-80},{40,-80}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation_L2.frame_b, Wing_L.frame_a) annotation (Line(
          points={{60,-80},{80,-80},{80,-20},{60,-20}},
          color={95,95,95},
          thickness=0.5));
      connect(fixedTranslation_R2.frame_b, Wing_R.frame_a) annotation (Line(
          points={{60,80},{80,80},{80,20},{60,20}},
          color={95,95,95},
          thickness=0.5));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=80, __Dymola_Algorithm="Dassl"));
    end Gull_1;
  end Flying_Gull;

  package RV_2AJ
    package RV_2AJ_Kinematics
      model Kinematics_Model
        inner Modelica.Mechanics.MultiBody.World world
          annotation (Placement(transformation(extent={{-80,-78},{-60,-58}})));
        Modelica.Mechanics.MultiBody.Joints.Revolute revolute(useAxisFlange=
              true, n(displayUnit="1") = {0,1,0}) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-50,-30})));
        Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(
          animateSphere=false,
          r(displayUnit="mm") = {0,0.1,0},
          m=10,
          width(displayUnit="mm") = 0.25) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-50,10})));
        Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape1(
          animateSphere=false,
          r(displayUnit="mm") = {0,0.25,0},
          m=4.72,
          width(displayUnit="mm") = 0.176) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={10,50})));
        Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape2(
          animateSphere=false,
          r(displayUnit="mm") = {0,0.16,0},
          m=2.28,
          width(displayUnit="mm") = 0.111) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={70,50})));
        Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(useAxisFlange=
              true) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-28,50})));
        Modelica.Mechanics.MultiBody.Joints.Revolute revolute2(useAxisFlange=
              true, n(displayUnit="1") = {1,0,0}) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={40,50})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a axis1
          "1-dim. rotational flange that drives the joint" annotation (
            Placement(transformation(extent={{-50,-10},{-30,10}}),
              iconTransformation(extent={{-50,-10},{-30,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a axis2
          "1-dim. rotational flange that drives the joint" annotation (
            Placement(transformation(extent={{20,60},{40,80}}),
              iconTransformation(extent={{20,60},{40,80}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a axis3
          "1-dim. rotational flange that drives the joint" annotation (
            Placement(transformation(extent={{-50,-90},{-30,-70}}),
              iconTransformation(extent={{-50,-90},{-30,-70}})));
      equation
        connect(world.frame_b, revolute.frame_a) annotation (Line(
            points={{-60,-68},{-50,-68},{-50,-40}},
            color={95,95,95},
            thickness=0.5));
        connect(revolute.frame_b, bodyShape.frame_a) annotation (Line(
            points={{-50,-20},{-50,0}},
            color={95,95,95},
            thickness=0.5));
        connect(revolute1.frame_a, bodyShape.frame_b) annotation (Line(
            points={{-38,50},{-50,50},{-50,20}},
            color={95,95,95},
            thickness=0.5));
        connect(revolute1.frame_b, bodyShape1.frame_a) annotation (Line(
            points={{-18,50},{0,50}},
            color={95,95,95},
            thickness=0.5));
        connect(bodyShape1.frame_b, revolute2.frame_a) annotation (Line(
            points={{20,50},{30,50}},
            color={95,95,95},
            thickness=0.5));
        connect(bodyShape2.frame_a, revolute2.frame_b) annotation (Line(
            points={{60,50},{50,50}},
            color={95,95,95},
            thickness=0.5));
        connect(revolute1.axis, axis1) annotation (Line(points={{-28,60},{-26,
                60},{-26,0},{-40,0}}, color={0,0,0}));
        connect(revolute2.axis, axis2)
          annotation (Line(points={{40,60},{40,70},{30,70}}, color={0,0,0}));
        connect(revolute.axis, axis3) annotation (Line(points={{-60,-30},{-12,
                -30},{-12,-80},{-40,-80}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics
              ={Rectangle(
                extent={{-100,-10},{20,-70}},
                lineColor={0,0,0},
                fillColor={244,125,35},
                fillPattern=FillPattern.HorizontalCylinder), Rectangle(
                extent={{-49.4974,9.89947},{49.4974,-9.89947}},
                lineColor={0,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={0,140,72},
                origin={-5.99995,34},
                rotation=45)}), Diagram(coordinateSystem(preserveAspectRatio=
                  false)));
      end Kinematics_Model;
    end RV_2AJ_Kinematics;

    package RV_2AJ_Control
      model Control_Model
        Modelica.Blocks.Math.UnitConversions.From_deg from_deg
          annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        Modelica.Mechanics.Rotational.Sources.Position position
          annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
        Modelica.Blocks.Interfaces.RealInput u1
                    "Connector of Real input signal to be converted"
          annotation (Placement(transformation(extent={{-128,-20},{-88,20}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{40,-10},{60,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                          "Flange of right shaft"
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      equation
        connect(position.phi_ref, from_deg.y)
          annotation (Line(points={{-10,0},{-39,0}}, color={0,0,127}));
        connect(from_deg.u, u1)
          annotation (Line(points={{-62,0},{-108,0}}, color={0,0,127}));
        connect(position.flange, inertia.flange_a)
          annotation (Line(points={{12,0},{40,0}}, color={0,0,0}));
        connect(inertia.flange_b, flange_b1)
          annotation (Line(points={{60,0},{100,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics
              ={Rectangle(
                extent={{-88,20},{90,-20}},
                lineColor={0,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={255,255,0})}), Diagram(coordinateSystem(
                preserveAspectRatio=false)));
      end Control_Model;
    end RV_2AJ_Control;

    package TestBench
      model Test_Model
        RV_2AJ_Kinematics.Kinematics_Model kinematics_Model
          annotation (Placement(transformation(extent={{-20,-80},{100,40}})));
        RV_2AJ_Control.Control_Model control_Model
          annotation (Placement(transformation(extent={{-60,-100},{-20,-60}})));
        RV_2AJ_Control.Control_Model control_Model1
          annotation (Placement(transformation(extent={{-40,-20},{0,20}})));
        RV_2AJ_Control.Control_Model control_Model2
          annotation (Placement(transformation(extent={{-20,40},{20,80}})));
        Modelica.Blocks.Sources.TimeTable timeTable(table=[0,0; 15,70; 25,45])
          annotation (Placement(transformation(extent={{-70,50},{-50,70}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=30*sin(0.5*time))
          annotation (Placement(transformation(extent={{-100,-10},{-60,10}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=170,
          duration=14,
          startTime=8)
          annotation (Placement(transformation(extent={{-100,-90},{-80,-70}})));
      equation
        connect(control_Model.flange_b1, kinematics_Model.axis3) annotation (
            Line(points={{-20,-80},{16,-80},{16,-68}}, color={0,0,0}));
        connect(control_Model1.flange_b1, kinematics_Model.axis1)
          annotation (Line(points={{0,0},{16,0},{16,-20}}, color={0,0,0}));
        connect(control_Model2.flange_b1, kinematics_Model.axis2)
          annotation (Line(points={{20,60},{58,60},{58,22}}, color={0,0,0}));
        connect(timeTable.y, control_Model2.u1)
          annotation (Line(points={{-49,60},{-21.6,60}}, color={0,0,127}));
        connect(realExpression.y, control_Model1.u1)
          annotation (Line(points={{-58,0},{-41.6,0}}, color={0,0,127}));
        connect(control_Model.u1, ramp.y)
          annotation (Line(points={{-61.6,-80},{-79,-80}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=30, __Dymola_Algorithm="Dassl"));
      end Test_Model;
    end TestBench;
  end RV_2AJ;

  package Expandable_Connector
    model Connectors
      Modelica.Blocks.Sources.Sine sine
        annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
      Modelica.Blocks.Sources.Ramp ramp
        annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder
        annotation (Placement(transformation(extent={{60,50},{80,70}})));
      Modelica.Blocks.Continuous.Integrator integrator
        annotation (Placement(transformation(extent={{60,0},{80,20}})));
      Modelica.Blocks.Continuous.Derivative derivative
        annotation (Placement(transformation(extent={{60,-42},{80,-22}})));
      Modelica.Blocks.Continuous.PI PI
        annotation (Placement(transformation(extent={{60,-80},{80,-60}})));
      Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces.ControlBus
        controlBus annotation (Placement(transformation(extent={{-20,-20},{20,
                20}}), iconTransformation(extent={{-266,16},{-246,36}})));
      Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces.ControlBus
        controlBus1 annotation (Placement(transformation(extent={{-20,-60},{20,
                -20}}), iconTransformation(extent={{-266,16},{-246,36}})));
    equation
      connect(controlBus.Signal_1, sine.y) annotation (Line(
          points={{0,0},{-28,0},{-28,50},{-59,50}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(controlBus.Signal_2, step.y) annotation (Line(
          points={{0,0},{-59,0}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(controlBus.Signal_3, ramp.y) annotation (Line(
          points={{0,0},{-20,0},{-20,-50},{-59,-50}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(controlBus.Signal_1, firstOrder.u) annotation (Line(
          points={{0,0},{20,0},{20,60},{58,60}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      connect(controlBus.Signal_2, integrator.u) annotation (Line(
          points={{0,0},{29,0},{29,10},{58,10}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      connect(controlBus.Signal_3, derivative.u) annotation (Line(
          points={{0,0},{40,0},{40,-32},{58,-32}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      connect(controlBus, controlBus1) annotation (Line(
          points={{0,0},{0,-40}},
          color={255,204,51},
          thickness=0.5));
      connect(controlBus1.Signal_3, PI.u) annotation (Line(
          points={{0,-40},{29,-40},{29,-70},{58,-70}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Connectors;

    model Connectors_Add
      Modelica.Blocks.Sources.Sine sine
        annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-80,10},{-60,30}})));
      Modelica.Blocks.Sources.Ramp ramp
        annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder
        annotation (Placement(transformation(extent={{60,70},{80,90}})));
      Modelica.Blocks.Continuous.Integrator integrator
        annotation (Placement(transformation(extent={{60,20},{80,40}})));
      Modelica.Blocks.Continuous.Derivative derivative
        annotation (Placement(transformation(extent={{60,-22},{80,-2}})));
      Modelica.Blocks.Continuous.PI PI
        annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
      Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces.ControlBus
        controlBus annotation (Placement(transformation(extent={{-20,0},{20,40}}),
            iconTransformation(extent={{-266,16},{-246,36}})));
      Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces.ControlBus
        controlBus1 annotation (Placement(transformation(extent={{-20,-40},{20,
                0}}), iconTransformation(extent={{-266,16},{-246,36}})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-70})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-80,-100},{-60,-80}})));
      Modelica.Electrical.Analog.Basic.Resistor resistor annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={20,-70})));
    equation
      connect(controlBus.Signal_1, sine.y) annotation (Line(
          points={{0,20},{-28,20},{-28,70},{-59,70}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(controlBus.Signal_2, step.y) annotation (Line(
          points={{0,20},{-59,20}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(controlBus.Signal_3, ramp.y) annotation (Line(
          points={{0,20},{-20,20},{-20,-30},{-59,-30}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{6,3},{6,3}},
          horizontalAlignment=TextAlignment.Left));
      connect(controlBus.Signal_1, firstOrder.u) annotation (Line(
          points={{0,20},{30,20},{30,80},{58,80}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      connect(controlBus.Signal_2, integrator.u) annotation (Line(
          points={{0,20},{39,20},{39,30},{58,30}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      connect(controlBus.Signal_3, derivative.u) annotation (Line(
          points={{0,20},{40,20},{40,-12},{58,-12}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      connect(controlBus, controlBus1) annotation (Line(
          points={{0,20},{0,-20}},
          color={255,204,51},
          thickness=0.5));
      connect(controlBus1.Signal_3, PI.u) annotation (Line(
          points={{0,-20},{29,-20},{29,-50},{58,-50}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}},
          horizontalAlignment=TextAlignment.Right));
      connect(ground.p, constantVoltage.n)
        annotation (Line(points={{-70,-80},{-40,-80}}, color={0,0,255}));
      connect(ground.p, resistor.n)
        annotation (Line(points={{-70,-80},{20,-80}}, color={0,0,255}));
      connect(controlBus.Pin_R, constantVoltage.p) annotation (Line(
          points={{0,20},{-16,20},{-16,-48},{-40,-48},{-40,-60}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-3,6},{-3,6}},
          horizontalAlignment=TextAlignment.Right));
      connect(controlBus.Pin_R, resistor.p) annotation (Line(
          points={{0,20},{20,20},{20,-60}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-3,6},{-3,6}},
          horizontalAlignment=TextAlignment.Right));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
    end Connectors_Add;
  end Expandable_Connector;

  package Redeclaration
    model Circuit
      replaceable Modelica.Electrical.Analog.Basic.Inductor resistor
        constrainedby Modelica.Electrical.Analog.Interfaces.OnePort annotation
        (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={0,30})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={58,-2})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-50,-60},{-30,-40}})));
    equation
      connect(resistor.n, constantVoltage.p)
        annotation (Line(points={{10,30},{58,30},{58,8}}, color={0,0,255}));
      connect(constantVoltage.n, ground.p) annotation (Line(points={{58,-12},{
              58,-40},{-40,-40}}, color={0,0,255}));
      connect(resistor.p, ground.p) annotation (Line(points={{-10,30},{-40,30},
              {-40,-40}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Circuit;
  end Redeclaration;

  package Textual_Modeling

    model Pendulum
      parameter Real m=1;
      parameter Real L=1;
      parameter Real g=9.81;
      parameter Real J=m*L^2;
      Real phi(start=2.0);
      Real w;
    equation
      der(phi)=w;
      J*der(w)=-m*g*L*sin(phi);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
    end Pendulum;

    model Simple_Pendulum
      extends Pendulum_Data;
    equation
      theta_dot=der(theta);
      der(theta_dot)=-Modelica.Constants.g_n/L*sin(theta)-C/(m*L^2)*theta_dot;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
    end Simple_Pendulum;

    record Pendulum_Data
      // Parameter Declaration
      parameter Modelica.SIunits.Mass m=1;
      parameter Modelica.SIunits.Length L=2;
      parameter Modelica.SIunits.RotationalDampingConstant C=1;
      parameter Modelica.SIunits.Angle theta0=0;
      parameter Modelica.SIunits.AngularVelocity theta_dot0=0;

      // Variable Declaration
      Modelica.SIunits.Angle theta(start=theta0);
      Modelica.SIunits.AngularVelocity theta_dot(start=theta_dot0);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Pendulum_Data;

    package Function
      function Sweep_Parameter
        input Integer simCases=5 "Number of Simulations";
        input String modelToSimulate="DymolaIntroductionCourse.Textual_Modeling.Simple_Pendulum" "Model to Simple";
        input String parameterToSweep="theta0" "Parameter to sweep";
        input Real parameterStartValue=0 "Start value of parameter";
        input Real increment=1 "Increment of parameter between each simulation";
        input String variableToPlot="theta" "Variable to plot";
      algorithm
        translateModel(modelToSimulate);
        plotSetFilesToKeep(simCases);
        // Save simCases number of result in memory
       for i in 1:simCases loop
         simulateExtendedModel(
         modelToSimulate, method="dassl", stopTime=5, resultFile="Simple_Pendulum_"+String(i),
         initialNames={parameterToSweep},
         initialValues={parameterStartValue+(i-1)*increment},
         finalNames={variableToPlot});
       end for;
       plot(y={variableToPlot},plotInAll=true);
       // Plot variable variableToPlot from all open results
      end Sweep_Parameter;
    end Function;
  end Textual_Modeling;

  package Simple_Rocket
    model Rocket
      parameter String name;
      Real mass(start=1038.358);
      Real altitude(start=59404);
      Real velocity(start=-2003);
      Real acceleration;
      Real thrust;
      Real gravity;
      parameter Real massLossRate=0.000277;

    equation
      (thrust-mass*gravity)/mass=acceleration;
      der(mass)=-massLossRate*abs(thrust);
      der(altitude)=velocity;
      der(velocity)=acceleration;
    end Rocket;

    model Celestial_Body
     constant Real g=6.673e-11;
     parameter Real radius;
     parameter String name;
     parameter Real mass;
    end Celestial_Body;

    model Moon_Landing
      parameter Real force1=36350;
      parameter Real force2=1308;
    protected
      parameter Real thrustEndTime=210;
      parameter Real thrustDecreaseTime=43.2;
    public
      Rocket phoenix(name="phoenix");
      Celestial_Body moon(name="Moon", mass=7.382e22, radius=1.738e6);
    equation
      phoenix.thrust=if (time<thrustDecreaseTime) then force1
     else
      if (time<thrustEndTime) then force2
      else
        0;
        phoenix.gravity=moon.g*moon.mass/(phoenix.altitude+moon.radius)^2;
    end Moon_Landing;
  end Simple_Rocket;

  package Graphical_n_Textual_Model
    model DCMotor
      extends Modelica.Electrical.Analog.Interfaces.OnePort;
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
        annotation (Placement(transformation(extent={{-10,90},{10,110}}),
            iconTransformation(extent={{-10,90},{10,110}})));
      // Parameters
      parameter Modelica.SIunits.Inductance L=0.01 "Inductance of the DC Motor";
      parameter Modelica.SIunits.Resistance R=0.05 "Resistance of the DC Motor";
      parameter Modelica.SIunits.ElectricalTorqueConstant K=0.3 "Torque Constant of the DC Motor";
      parameter Modelica.SIunits.Inertia J=0.2 "Rotor Inertia of the DC Motor";

      // Variables
      Modelica.SIunits.AngularVelocity w "Angular Velocity of the DC Motor";

    equation
      der(flange_a.phi)=w;
      v=R*i+L*der(i)+K*w;
      flange_a.tau=-K*i+J*der(w);
      annotation (Icon(graphics={
            Ellipse(
              extent={{-100,-54},{100,-28}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={244,125,35}),
            Rectangle(
              extent={{-100,40},{100,-42}},
              lineColor={0,0,0},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={244,125,35}),
            Ellipse(
              extent={{-100,26},{100,52}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={244,125,35}),
            Rectangle(
              extent={{-10,100},{10,40}},
              lineColor={0,0,0},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={175,175,175})}));
    end DCMotor;

    model DCMotor_Test
      DCMotor dCMotor
        annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=2)
        annotation (Placement(transformation(extent={{16,56},{36,76}})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=100)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,-22})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-90,-82},{-70,-62}})));
    equation
      connect(dCMotor.flange_a, inertia.flange_a)
        annotation (Line(points={{0,20},{0,66},{16,66}}, color={0,0,0}));
      connect(dCMotor.p, constantVoltage.p)
        annotation (Line(points={{-20,0},{-80,0},{-80,-12}}, color={0,0,255}));
      connect(constantVoltage.n, ground.p)
        annotation (Line(points={{-80,-32},{-80,-62}}, color={0,0,255}));
      connect(dCMotor.n, ground.p) annotation (Line(points={{20,0},{80,0},{80,
              -62},{-80,-62}}, color={0,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=20, __Dymola_Algorithm="Dassl"));
    end DCMotor_Test;
  end Graphical_n_Textual_Model;

  package Algorithm
    model Bouncing_Ball "The Bouncing Ball Model"
      parameter Real g=9.81; //Gravitational Acceleration
      parameter Real c=0.9; //Elasticity Constant
      Real height(start=0), velocity(start=10);
    equation
      der(height)=velocity;
      der(velocity)=-g;
      when height < 0 then
        reinit(velocity, -c*velocity);
      end when;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end Bouncing_Ball;

    function Polynomial_Evaluator
      input Real A[:];  // Array, Size defined
                        // at function Call time
      input Real x=1.0; // Default Value 1.0 for x
      output Real sum;
    protected
      Real xpower;      // Local Variable xpower
    algorithm
      sum := 0;
      xpower := 1;
      for i in 1:size(A,1) loop
        sum := sum+A[i]*xpower;
        xpower := xpower*x;
      end for;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
              preserveAspectRatio=false)));
    end Polynomial_Evaluator;
  end Algorithm;

  package State_Machines

    model Example_1
      inner Integer i(start=0);
      block State1
        outer output Integer i;
      equation
        i=previous(i)+2;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%stateName",
                fontSize=10), Text(
                extent={{-100,100},{100,-50}},
                lineColor={0,0,0},
                fillPattern=FillPattern.VerticalCylinder,
                fillColor={175,175,175},
                textString="%stateText")}),
          __Dymola_state=true,
          showDiagram=true,
          singleInstance=true);
      end State1;
      State1 state1 annotation (Placement(transformation(extent={{-20,20},{60,
                40}})));
      block State2
        outer output Integer i;
      equation
        i=previous(i)-1;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%stateName",
                fontSize=10), Text(
                extent={{-100,100},{100,-4}},
                lineColor={0,0,0},
                fillPattern=FillPattern.VerticalCylinder,
                fillColor={175,175,175},
                textString="%stateText")}),
          __Dymola_state=true,
          showDiagram=true,
          singleInstance=true);
      end State2;
      State2 state2 annotation (Placement(transformation(extent={{-20,-40},{60,-20}})));
    equation
      initialState(state1) annotation (Line(
          points={{28,42},{28,62},{28,80}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier,
          arrow={Arrow.Filled,Arrow.None}));
      transition(
        state1,
        state2,
        i > 10,
        immediate=false,
        reset=true,
        synchronize=false,
        priority=1) annotation (Line(
          points={{24,18},{24,-18}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier), Text(
          string="%condition",
          extent={{-4,-4},{-4,-10}},
          fontSize=10,
          textStyle={TextStyle.Bold},
          horizontalAlignment=TextAlignment.Right));
      transition(
        state2,
        state1,
        i < 1,
        immediate=false) annotation (Line(
          points={{26,-42},{26,-80},{-80,-80},{-80,80},{-2,80},{-2,42}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier), Text(
          string="%condition",
          extent={{-4,-4},{-4,-10}},
          fontSize=10,
          textStyle={TextStyle.Bold},
          horizontalAlignment=TextAlignment.Right));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
              preserveAspectRatio=false), graphics={Text(
              extent={{12,98},{96,74}},
              lineColor={0,0,0},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={175,175,175},
              textString="%declarations")}),
        experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
    end Example_1;

    model Example_2
      inner Integer i(start=0);
      block State1
        outer output Integer i;
        Increment add(increment=2) annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
        Modelica.Blocks.Interfaces.IntegerInput u1
                                                  "Integer input signal"
          annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
        Modelica.Blocks.Interfaces.IntegerOutput y1 "Integer output signal"
          annotation (Placement(transformation(extent={{96,-10},{116,10}})));
      equation
        i=previous(i)+2;
        connect(add.u, u1) annotation (Line(points={{-28,0},{-106,0}}, color={255,127,0}));
        connect(add.y, y1) annotation (Line(points={{24,0},{106,0}}, color={255,127,0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%stateName",
                fontSize=10)}),
          __Dymola_state=true,
          showDiagram=true,
          singleInstance=true);
      end State1;
      State1 state1 annotation (Placement(transformation(extent={{-60,20},{-20,60}})));
      block State2
        outer output Integer i;
        Modelica.Blocks.Interfaces.IntegerInput u1
                                                  "Integer input signal"
          annotation (Placement(transformation(extent={{-128,-20},{-88,20}})));
        Modelica.Blocks.Interfaces.IntegerOutput y1
                                                   "Integer output signal"
          annotation (Placement(transformation(extent={{96,-10},{116,10}})));
        Increment sub(increment=-1) annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
      equation
        i=previous(i)-1;
        connect(sub.u, u1) annotation (Line(points={{-28,0},{-108,0}}, color={255,127,0}));
        connect(sub.y, y1) annotation (Line(points={{24,0},{106,0}}, color={255,127,0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                textString="%stateName",
                fontSize=10)}),
          __Dymola_state=true,
          showDiagram=true,
          singleInstance=true);
      end State2;
      State2 state2 annotation (Placement(transformation(extent={{-60,-60},{-20,-20}})));
      Prev prev annotation (Placement(transformation(extent={{40,-20},{80,20}})));
      Modelica.Blocks.Interfaces.IntegerOutput I "Integer output signal"
        annotation (Placement(transformation(extent={{4,-10},{24,10}})));
    equation
      initialState(state1) annotation (Line(
          points={{-36,62},{-36,76},{-36,94}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier,
          arrow={Arrow.Filled,Arrow.None}));
      transition(state1,state2,I > 10,
        immediate=false,
        reset=true,
        synchronize=false,
        priority=1) annotation (Line(
          points={{-38,18},{-38,-18}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier), Text(
          string="%condition",
          extent={{-4,-4},{-4,-10}},
          fontSize=10,
          textStyle={TextStyle.Bold},
          horizontalAlignment=TextAlignment.Right));
      transition(state2,state1,I
          < 1,
        immediate=false) annotation (Line(
          points={{-37,-62},{-38,-80},{-90,-80},{-90,80},{-50,80},{-51,62}},
          color={175,175,175},
          thickness=0.25,
          smooth=Smooth.Bezier), Text(
          string="%condition",
          extent={{-4,-4},{-4,-10}},
          fontSize=10,
          textStyle={TextStyle.Bold},
          horizontalAlignment=TextAlignment.Right));
      connect(prev.y, state2.u1) annotation (Line(points={{84,0},{92,0},{92,-90},{-72,-90},{-72,-40},{-61.6,-40}},
            color={255,127,0}));
      connect(prev.y, state1.u1) annotation (Line(points={{84,0},{92,0},{92,-90},{-78,-90},{-78,40},{-61.2,40}},
            color={255,127,0}));
      connect(state1.y1,I)  annotation (Line(points={{-18.8,40},{-12,40},{-12,0},{14,0}}, color={255,127,0}));
      connect(prev.u,I)  annotation (Line(points={{32,0},{14,0}}, color={255,127,0}));
      connect(state2.y1,I)
        annotation (Line(points={{-18.8,-40},{-12,-40},{-12,0},{14,0}}, color={255,127,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(
              preserveAspectRatio=false)),
        experiment(StopTime=25, __Dymola_Algorithm="Dassl"));
    end Example_2;

    block Increment
      extends Modelica.Blocks.Interfaces.PartialIntegerSISO;
      parameter Integer increment;
    equation
      y=u+increment;
    end Increment;

    block Prev
      extends Modelica.Blocks.Interfaces.PartialIntegerSISO;

    equation
      y=previous(u);
    end Prev;
  end State_Machines;
  annotation (uses(Modelica(version="3.2.3")));
end DymolaIntroductionCourse;
