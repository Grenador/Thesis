within ;
package BM_ABC "Building Model A,B,C Blocks"
  package Data "Data for transient thermal building simulation"

  extends Modelica.Icons.MaterialPropertiesPackage;

    package Materials "Library of construction materials"

    extends Modelica.Icons.MaterialPropertiesPackage;

      record Air_NV1 =
                   IDEAS.Buildings.Data.Interfaces.Material (
          k=0.33,
          c=1008,
          rho=1.23,
          epsSw=0,
          epsLw=0,
          gas=true,
          mhu=18.3*10e-6) "Non-Ventilated Air Cavity of 6cm length";
      record Air_NV2 =
                   IDEAS.Buildings.Data.Interfaces.Material (
          k=0.111,
          c=1008,
          rho=1.23,
          epsSw=0,
          epsLw=0,
          gas=true,
          mhu=18.3*10e-6) "Non-Ventilated Air Cavity of 2cm length";
    end Materials;

    package Glazing "Library of building glazing systems"

    extends Modelica.Icons.MaterialPropertiesPackage;

      record SouthWindow = IDEAS.Buildings.Data.Interfaces.Glazing (
          nLay=3,
          mats={IDEAS.Buildings.Data.Materials.Glass(d=0.004),
              IDEAS.Buildings.Data.Materials.Air(d=0.006),
              IDEAS.Buildings.Data.Materials.Glass(d=0.004)},
          SwTrans=[0, 0.726; 10, 0.725; 20, 0.722; 30, 0.716; 40, 0.702; 50,
              0.670; 60, 0.601; 70, 0.458; 80, 0.220; 90, 0.000],
          SwAbs=[0, 0.080, 0.0, 0.061; 10, 0.080, 0.0, 0.061; 20, 0.081, 0.0,
              0.062; 30, 0.084, 0.0, 0.063; 40, 0.087, 0.0, 0.065; 50, 0.092,
              0.0, 0.066; 60, 0.098, 0.0, 0.065; 70, 0.106, 0.0, 0.059; 80,
              0.109, 0.0, 0.044; 90, 0.000, 0.0, 0.000],
          SwTransDif=0.624,
          SwAbsDif={0.091,0.0,0.062},
          U_value=3.886,
          g_value=0.77) "South Window (U=3.886 W/m2K, g=0.77)";
    end Glazing;

    package Constructions "Library of building envelope constructions"

    extends Modelica.Icons.MaterialPropertiesPackage;

      record FloorAboveBasement

        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=4,
              mats={IDEAS.Buildings.Data.Materials.Tile(d=0.01905),
              IDEAS.Buildings.Data.Materials.Concrete(d=0.1),
              IDEAS.Buildings.Data.Insulation.Pur(d=0.07),
              IDEAS.Buildings.Data.Materials.Concrete(d=0.08)});

      end FloorAboveBasement;

      record CeilingRoof

        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=4,
              mats={IDEAS.Buildings.Data.Materials.Concrete(d=0.1),
              IDEAS.Buildings.Data.Insulation.Pur(d=0.0235),
              IDEAS.Buildings.Data.Materials.Concrete(d=0.1265),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
      end CeilingRoof;

      record EastWest_InsulatedWall
      extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=5,
              final mats={IDEAS.Buildings.Data.Insulation.Pur(d=0.0496),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              BM_ABC.Data.Materials.Air_NV1(d=0.06),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
      end EastWest_InsulatedWall;

      record SouthWall
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=4,
              final mats={IDEAS.Buildings.Data.Materials.BrickLi(d=0.09),
              BM_ABC.Data.Materials.Air_NV1(d=0.06),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
      end SouthWall;

      record NorthWall
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=5,
              final mats={IDEAS.Buildings.Data.Insulation.Pur(d=0.0533),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.09),
              BM_ABC.Data.Materials.Air_NV1(d=0.06),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
      end NorthWall;

      record EastWest_NonInsulatedWall
          extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=4,
              final mats={IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              BM_ABC.Data.Materials.Air_NV1(d=0.06),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
      end EastWest_NonInsulatedWall;

      record Internal_Wall1 "Internall Wall between Apartments"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=5,
              mats={IDEAS.Buildings.Data.Materials.Gypsum(d=0.01),
              IDEAS.Buildings.Data.Materials.BrickLi(d= 0.14),
              BM_ABC.Data.Materials.Air_NV2(d=0.02),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});

      end Internal_Wall1;

      record Internal_Wall2 "Internall Wall in the Apartments type 1 Big Brick"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=3,
              mats={IDEAS.Buildings.Data.Materials.Gypsum(d=0.01),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.14),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});

      end Internal_Wall2;

      record Internal_Wall3
        "Internall Wall in the Apartments type 1 Small Brick"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=3,
              mats={IDEAS.Buildings.Data.Materials.Gypsum(d=0.01),
              IDEAS.Buildings.Data.Materials.BrickLi(d=0.09),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});

      end Internal_Wall3;

      record Normal_CeilingFloor "Normal Ceiling-Floor between Apartments"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=3,
              mats={IDEAS.Buildings.Data.Materials.Tile(d=0.01905),
              IDEAS.Buildings.Data.Materials.Concrete(d=0.25),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});

      end Normal_CeilingFloor;

      record BasementWall_1 "Basement Wall 39cm"

        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=2,
              mats={IDEAS.Buildings.Validation.Data.Materials.ConcreteBlock(d=0.39),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
        annotation ();
      end BasementWall_1;

      record BasementWall_2 "Basement Wall 29cm"

        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=2,
              mats={IDEAS.Buildings.Validation.Data.Materials.ConcreteBlock(d=0.29),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
        annotation ();
      end BasementWall_2;

      record BasementWall_3 "Basement Wall Internal 19cm"

        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=3,
              mats={IDEAS.Buildings.Data.Materials.Gypsum(d=0.01),
              IDEAS.Buildings.Validation.Data.Materials.ConcreteBlock(d=0.19),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
        annotation ();
      end BasementWall_3;

      record BasementWall_4 "Basement Wall Internal 29cm"

        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=3,
              mats={IDEAS.Buildings.Data.Materials.Gypsum(d=0.01),
              IDEAS.Buildings.Validation.Data.Materials.ConcreteBlock(d=0.29),
              IDEAS.Buildings.Data.Materials.Gypsum(d=0.01)});
        annotation ();
      end BasementWall_4;

      record LightRoof "Light roof"

        extends IDEAS.Buildings.Data.Interfaces.Construction(
          nLay=3,
          mats={IDEAS.Buildings.Validation.Data.Materials.Roofdeck(d=0.019),
          IDEAS.Buildings.Data.Insulation.Pur(d=0.01),
              IDEAS.Buildings.Validation.Data.Materials.PlasterBoard(d=0.010)});

      end LightRoof;
    annotation (Documentation(info="<html>
<p>By convention the last material layer of a construction type is connected to propsBus_a of the building component.</p>
</html>"));
    end Constructions;

    package Components

      model Zone "thermal building zone"
        import Buildings;

        extends IDEAS.Buildings.Components.Interfaces.StateZone;
        extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations(redeclare
            package Medium =
                     IDEAS.Experimental.Media.AirPTDecoupled);

        outer Modelica.Fluid.System system
          annotation (Placement(transformation(extent={{-80,80},{-60,100}})));
        parameter Boolean allowFlowReversal=system.allowFlowReversal
          "= true to allow flow reversal in zone, false restricts to design direction (port_a -> port_b)."
          annotation(Dialog(tab="Assumptions"));

        parameter Modelica.SIunits.Volume V "Total zone air volume";
        parameter Real n50(min=0.01)=0.4
          "n50 value cfr airtightness, i.e. the ACH at a pressure diffence of 50 Pa";
        parameter Real corrCV=5
          "Multiplication factor for the zone air capacity";

        parameter Boolean linear=true
          "Linearized computation of long wave radiation";

        final parameter Modelica.SIunits.Power QInf_design=1012*1.204*V/3600*n50/20*(273.15
             + 21 - sim.Tdes)
          "Design heat losses from infiltration at reference outdoor temperature";
        final parameter Modelica.SIunits.MassFlowRate m_flow_nominal = 0.1*1.224*V/3600;
        final parameter Modelica.SIunits.Power QRH_design=A*fRH
          "Additional power required to compensate for the effects of intermittent heating";
        parameter Real fRH=11
          "Reheat factor for calculation of design heat load, (EN 12831, table D.10 Annex D)"
                                                                                              annotation(Dialog(group="Design heat load"));
        parameter Modelica.SIunits.Area A = 0 "Total conditioned floor area" annotation(Dialog(group="Design heat load"));

        Modelica.SIunits.Power QTra_design=sum(propsBus.QTra_design)
          "Total design transmission heat losses for the zone";
        final parameter Modelica.SIunits.Power Q_design( fixed=false)
          "Total design heat losses for the zone";

        Modelica.SIunits.Temperature TAir=senTem.T;
        Modelica.SIunits.Temperature TStar=radDistr.TRad;

      protected
        IDEAS.Buildings.Components.BaseClasses.ZoneLwGainDistribution radDistr(final
            nSurf=nSurf) "distribution of radiative internal gains" annotation (
            Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={-54,-44})));
        IDEAS.Buildings.Components.BaseClasses.AirLeakage airLeakage(
          redeclare package Medium = Medium,
          m_flow_nominal=V/3600*n50/20,
          V=V,
          n50=n50,
          allowFlowReversal=allowFlowReversal,
          show_T=false)
          annotation (Placement(transformation(extent={{40,30},{60,50}})));
        IDEAS.Buildings.Components.BaseClasses.ZoneLwDistribution radDistrLw(final
            nSurf=nSurf, final linear=linear)
          "internal longwave radiative heat exchange" annotation (Placement(
              transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-54,-10})));
        Modelica.Blocks.Math.Sum summation(nin=2, k={0.5,0.5})
          annotation (Placement(transformation(extent={{0,-66},{12,-54}})));
        IDEAS.Fluid.MixingVolumes.MixingVolume vol(
          V=V,
          m_flow_nominal=m_flow_nominal,
          nPorts=if allowFlowReversal then 4 else 2,
          energyDynamics=energyDynamics,
          massDynamics=massDynamics,
          p_start=p_start,
          T_start=T_start,
          X_start=X_start,
          C_start=C_start,
          C_nominal=C_nominal,
          allowFlowReversal=allowFlowReversal,
          mSenFac=corrCV,
          redeclare package Medium = Modelica.Media.Air.SimpleAir) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-6,32})));
      public
        IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out(redeclare package Medium
            = Modelica.Media.Air.SimpleAir)
          annotation (Placement(transformation(extent={{-30,90},{-10,110}})));
        IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In(redeclare package Medium
            = Modelica.Media.Air.SimpleAir)
          annotation (Placement(transformation(extent={{10,90},{30,110}})));
      protected
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTem
          annotation (Placement(transformation(extent={{0,-28},{-16,-12}})));

      initial equation
        Q_design=QInf_design+QRH_design+QTra_design; //Total design load for zone (additional ventilation losses are calculated in the ventilation system)
      equation

        connect(radDistr.radGain, gainRad) annotation (Line(
            points={{-50.2,-54},{-50,-54},{-50,-72},{80,-72},{80,-60},{100,-60}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(propsBus[:].surfRad, radDistrLw.port_a) annotation (Line(
            points={{-100,40},{-74,40},{-74,-26},{-54,-26},{-54,-20}},
            color={191,0,0},
            smooth=Smooth.None));

        connect(summation.y, TSensor) annotation (Line(
            points={{12.6,-60},{59.3,-60},{59.3,0},{106,0}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(radDistr.TRad, summation.u[1]) annotation (Line(
            points={{-44,-44},{-22,-44},{-22,-60.6},{-1.2,-60.6}},
            color={0,0,127},
            smooth=Smooth.None));

        connect(propsBus.area, radDistr.area) annotation (Line(
            points={{-100,40},{-82,40},{-82,-40},{-64,-40}},
            color={127,0,0},
            smooth=Smooth.None), Text(
            string="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(propsBus.area, radDistrLw.A) annotation (Line(
            points={{-100,40},{-82,40},{-82,-14},{-64,-14}},
            color={127,0,0},
            smooth=Smooth.None), Text(
            string="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(propsBus.epsLw, radDistrLw.epsLw) annotation (Line(
            points={{-100,40},{-82,40},{-82,-10},{-64,-10}},
            color={127,0,0},
            smooth=Smooth.None), Text(
            string="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(propsBus.epsLw, radDistr.epsLw) annotation (Line(
            points={{-100,40},{-82,40},{-82,-44},{-64,-44}},
            color={127,0,0},
            smooth=Smooth.None), Text(
            string="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(propsBus.epsSw, radDistr.epsSw) annotation (Line(
            points={{-100,40},{-82,40},{-82,-48},{-64,-48}},
            color={127,0,0},
            smooth=Smooth.None), Text(
            string="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(vol.heatPort, gainCon) annotation (Line(
            points={{4,32},{10,32},{10,-30},{100,-30}},
            color={191,0,0},
            smooth=Smooth.None));

      for i in 1:nSurf loop
        connect(radDistr.iSolDir, propsBus[i].iSolDir) annotation (Line(
            points={{-58,-54},{-58,-80},{-100,-80},{-100,40}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(radDistr.iSolDif, propsBus[i].iSolDif) annotation (Line(
            points={{-54,-54},{-54,-76},{-100,-76},{-100,40}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(propsBus[i].surfCon, vol.heatPort) annotation (Line(
            points={{-100,40},{-46,40},{-46,12},{10,12},{10,32},{4,32}},
            color={191,0,0},
            smooth=Smooth.None));
      end for;
        connect(flowPort_In, vol.ports[1]) annotation (Line(
            points={{20,100},{20,42},{-6,42}},
            color={0,128,255},
            smooth=Smooth.None));
        connect(flowPort_Out, vol.ports[2]) annotation (Line(
            points={{-20,100},{-20,42},{-6,42}},
            color={0,128,255},
            smooth=Smooth.None));
        connect(senTem.port, gainCon) annotation (Line(
            points={{0,-20},{10,-20},{10,-30},{100,-30}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(senTem.T, summation.u[2]) annotation (Line(
            points={{-16,-20},{-18,-20},{-18,-59.4},{-1.2,-59.4}},
            color={0,0,127},
            smooth=Smooth.None));
            if allowFlowReversal then
        connect(airLeakage.port_a, vol.ports[4]) annotation (Line(
            points={{40,40},{16,40},{16,42},{-6,42}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(airLeakage.port_b, vol.ports[3]) annotation (Line(
            points={{60,40},{70,40},{70,14},{-32,14},{-32,42},{-6,42}},
            color={0,127,255},
            smooth=Smooth.None));
            else
        connect(airLeakage.port_a, vol.ports[2]) annotation (Line(
            points={{40,40},{16,40},{16,42},{-6,42}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(airLeakage.port_b, vol.ports[1]) annotation (Line(
            points={{60,40},{70,40},{70,14},{-32,14},{-32,42},{-6,42}},
            color={0,127,255},
            smooth=Smooth.None));
            end if;
        connect(radDistr.radSurfTot, radDistrLw.port_a) annotation (Line(
            points={{-54,-34},{-54,-20}},
            color={191,0,0},
            smooth=Smooth.None));

      for i in 1:nSurf loop
      connect(sim.weaBus, propsBus[i].weaBus) annotation (Line(
             points={{-88.6,97.2},{-88.6,100},{-100,100},{-100,40}},
             color={255,204,51},
             thickness=0.5,
             smooth=Smooth.None));
      end for;

        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
               graphics),
          Documentation(info="<html>
<p><h4><font color=\"#008000\">General description</font></h4></p>
<p><h5>Goal</h5></p>
<p>Also the thermal response of a zone can be divided into a convective, longwave radiative and shortwave radiative process influencing both thermal comfort in the depicted zone as well as the response of adjacent wall structures.</p>
<p><h5>Description</h5></p>
<p>The air within the zone is modeled based on the assumption that it is well-stirred, i.e. it is characterized by a single uniform air temperature. This is practically accomplished with the mixing caused by the air distribution system. The convective gains and the resulting change in air temperature T_{a} of a single thermal zone can be modeled as a thermal circuit. The resulting heat balance for the air node can be described as c_{a}.V_{a}.dT_{a}/dt = som(Q_{ia}) + sum(h_{ci}.A_{si}.(T_{a}-T_{si})) + sum(m_{az}.(h_{a}-h_{az})) + m_{ae}(h_{a}-h_{ae}) + m_{sys}(h_{a}-h_{sys}) wherefore h_{a} is the specific air enthalpy and where T_{a} is the air temperature of the zone, c_{a} is the specific heat capacity of air at constant pressure, V_{a} is the zone air volume, Q_{a} is a convective internal load, R_{si} is the convective surface resistance of surface s_{i}, A_{si} is the area of surface s_{i}, T_{si} the surface temperature of surface s_{i}, m_{az} is the mass flow rate between zones, m_{ae} is the mass flow rate between the exterior by natural infiltrationa and m_{sys} is the mass flow rate provided by the ventilation system. </p>
<p>Infiltration and ventilation systems provide air to the zones, undesirably or to meet heating or cooling loads. The thermal energy provided to the zone by this air change rate can be formulated from the difference between the supply air enthalpy and the enthalpy of the air leaving the zone <img src=\"modelica://IDEAS/Images/equations/equation-jiSQ22c0.png\" alt=\"h_a\"/>. It is assumed that the zone supply air mass flow rate is exactly equal to the sum of the air flow rates leaving the zone, and all air streams exit the zone at the zone mean air temperature. The moisture dependence of the air enthalpy is neglected.</p>
<p>A multiplier for the zone capacitance f_{ca} is included. A f_{ca} equaling unity represents just the capacitance of the air volume in the specified zone. This multiplier can be greater than unity if the zone air capacitance needs to be increased for stability of the simulation. This multiplier increases the capacitance of the air volume by increasing the zone volume and can be done for numerical reasons or to account for the additional capacitances in the zone to see the effect on the dynamics of the simulation. This multiplier is constant throughout the simulation and is set to 5.0 if the value is not defined <a href=\"IDEAS.Buildings.UsersGuide.References\">[Masy 2008]</a>.</p>
<p>The exchange of longwave radiation in a zone has been previously described in the building component models and further considering the heat balance of the interior surface. Here, an expression based on <i>radiant interchange configuration factors</i> or <i>view factors</i> is avoided based on a delta-star transformation and by definition of a <i>radiant star temperature</i> T_{rs}. Literature <a href=\"IDEAS.Buildings.UsersGuide.References\">[Liesen 1997]</a> shows that the overall model is not significantly sensitive to this assumption. ThisT_{rs} can be derived from the law of energy conservation in the radiant star node as sum(Q_{si,rs}) must equal zero. Long wave radiation from internal sources are dealt with by including them in the heat balance of the radiant star node resulting in a diffuse distribution of the radiative source.</p>
<p>Transmitted shortwave solar radiation is distributed over all surfaces in the zone in a prescribed scale. This scale is an input value which may be dependent on the shape of the zone and the location of the windows, but literature <a href=\"IDEAS.Buildings.UsersGuide.References\">[Liesen 1997]</a> shows that the overall model is not significantly sensitive to this assumption.</p>
<p><h4><font color=\"#008000\">Validation </font></h4></p>
<p>By means of the <code>BESTEST.mo</code> examples in the <code>Validation.mo</code> package.</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                              graphics));
      end Zone;
    end Components;

  end Data;

  package Interfaces

  extends Modelica.Icons.InterfacesPackage;
  import SI = Modelica.SIunits;

    model Building

      inner IDEAS.SimInfoManager sim(lat=0.88801968369036, lon=
            0.081801662178797)
        annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

      replaceable package Medium=IDEAS.Media.Water.Simple;

      parameter Boolean standAlone=true;
      parameter Boolean isDH=false
        "True if the building is connected to a DH grid";

      final parameter Modelica.SIunits.Temperature[building.nZones] T_start = ones(building.nZones)*293.15
        "Operative zonal start temperatures";
      final parameter Modelica.SIunits.Power[building.nZones] Q_design = building.Q_design+ventilationSystem.Q_design
        "Total design heat load for heating system based on heat losses";

      replaceable BM_ABC.Interfaces.BaseClasses.Structure building(
        redeclare package Medium = Modelica.Media.Air.SimpleAir)   constrainedby
        BM_ABC.Interfaces.BaseClasses.Structure(final T_start=T_start)
        "Building structure" annotation (Placement(transformation(extent={{-66,-10},
                {-36,10}})), choicesAllMatching=true);

      replaceable BaseClasses.HeatingSystem heatingSystem(
        nLoads=0,
        redeclare package Medium = IDEAS.Media.Water.Simple) constrainedby
        BM_ABC.Interfaces.BaseClasses.HeatingSystem(
        redeclare package Medium = Medium,
        final isDH=isDH,
        final nZones=building.nZones,
        final nEmbPorts=building.nEmb,
        final InInterface=InInterface,
        final Q_design=Q_design) "Thermal building heating system" annotation (
          Placement(transformation(extent={{-20,-10},{20,10}})), choicesAllMatching=
           true);

      replaceable BaseClasses.Occupant occupant constrainedby
        BM_ABC.Interfaces.BaseClasses.Occupant(nZones=building.nZones)
        "Building occupant" annotation (Placement(transformation(extent={{-20,-50},{
                20,-30}})), choicesAllMatching=true);

      replaceable BM_ABC.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid
        constrainedby BM_ABC.Interfaces.BaseClasses.CausalInhomeFeeder
        "Inhome low-voltage electricity grid system" annotation (Placement(
            transformation(extent={{32,-10},{52,10}})), __Dymola_choicesAllMatching=
           true);

      replaceable BaseClasses.VentilationSystem ventilationSystem(
        redeclare package Medium = Modelica.Media.Air.SimpleAir)
       constrainedby BM_ABC.Interfaces.BaseClasses.VentilationSystem(
          final nZones=building.nZones,
          final VZones=building.VZones) "Ventilation system" annotation (
          Placement(transformation(extent={{-20,20},{20,40}})),
          choicesAllMatching=true);

      Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin
        plugFeeder(v(re(start=230), im(start=0))) if not standAlone
        "Electricity connection to the district feeder"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Electrical.QuasiStationary.SinglePhase.Sources.VoltageSource
        voltageSource(
        f=50,
        V=230,
        phi=0) if standAlone annotation (Placement(transformation(
            extent={{-8,-8},{8,8}},
            rotation=90,
            origin={70,-12})));
      Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground if
        standAlone
        annotation (Placement(transformation(extent={{62,-40},{78,-24}})));

      IDEAS.Fluid.Interfaces.FlowPort_a flowPort_supply(redeclare package
          Medium =
            Medium) if isDH
        annotation (Placement(transformation(extent={{10,-110},{30,-90}})));
      IDEAS.Fluid.Interfaces.FlowPort_b flowPort_return(redeclare package
          Medium =
            Medium) if isDH
        annotation (Placement(transformation(extent={{-30,-110},{-10,-90}})));
      final parameter Boolean InInterface = true;

      inner Modelica.Fluid.System system
      annotation (Placement(transformation(extent={{-72,74},{-52,94}})));
    equation
      connect(heatingSystem.TSet, occupant.TSet) annotation (Line(
          points={{0,-10.2},{0,-30}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(building.heatPortEmb, heatingSystem.heatPortEmb) annotation (Line(
          points={{-37.5,5.06667},{-28,5.06667},{-28,6},{-20,6}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(building.heatPortCon, heatingSystem.heatPortCon) annotation (Line(
          points={{-37.5,2.4},{-28,2.4},{-28,2},{-20,2}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(building.heatPortCon, occupant.heatPortCon) annotation (Line(
          points={{-37.5,2.4},{-26,2.4},{-26,-38},{-20,-38}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(building.heatPortRad, heatingSystem.heatPortRad) annotation (Line(
          points={{-37.5,-0.266667},{-28,-0.266667},{-28,-2},{-20,-2}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(building.heatPortRad, occupant.heatPortRad) annotation (Line(
          points={{-37.5,-0.266667},{-30,-0.266667},{-30,-42},{-20,-42}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(building.TSensor, heatingSystem.TSensor) annotation (Line(
          points={{-37.2,-2.93333},{-28,-2.93333},{-28,-6},{-20.4,-6}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(building.TSensor, ventilationSystem.TSensor) annotation (Line(
          points={{-37.2,-2.93333},{-32,-2.93333},{-32,24},{-20.4,24}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));

      connect(ventilationSystem.plugLoad, inHomeGrid.nodeSingle) annotation (Line(
          points={{20,30},{26,30},{26,0},{32,0}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(heatingSystem.plugLoad, inHomeGrid.nodeSingle) annotation (Line(
          points={{20,0},{32,0}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(occupant.plugLoad, inHomeGrid.nodeSingle) annotation (Line(
          points={{20,-40},{26,-40},{26,0},{32,0}},
          color={85,170,255},
          smooth=Smooth.None));

      if standAlone then
        connect(voltageSource.pin_p, ground.pin) annotation (Line(
            points={{70,-20},{70,-24}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(inHomeGrid.pinSingle, voltageSource.pin_n) annotation (Line(
            points={{52,0},{70,0},{70,-4}},
            color={85,170,255},
            smooth=Smooth.None));
      else
        connect(inHomeGrid.pinSingle, plugFeeder) annotation (Line(
            points={{52,0},{100,0}},
            color={85,170,255},
            smooth=Smooth.None));
      end if;

      connect(heatingSystem.mDHW60C, occupant.mDHW60C) annotation (Line(
          points={{6,-10.2},{6,-30}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(ventilationSystem.flowPort_Out, building.flowPort_In) annotation (
          Line(
          points={{-20,28},{-50,28},{-50,8.53333}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(building.flowPort_Out, ventilationSystem.flowPort_In) annotation (
          Line(
          points={{-52,8.53333},{-52,32},{-20,32}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(heatingSystem.flowPort_return, flowPort_return) annotation (Line(
          points={{12,-10},{12,-24},{32,-24},{32,-60},{-20,-60},{-20,-100}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(heatingSystem.flowPort_supply, flowPort_supply) annotation (Line(
          points={{16,-10},{16,-20},{40,-20},{40,-70},{20,-70},{20,-100}},
          color={0,0,0},
          smooth=Smooth.None));
      annotation (Icon(graphics={
            Line(
              points={{60,22},{0,74},{-60,24},{-60,-46},{60,-46}},
              color={127,0,0},
              smooth=Smooth.None),
            Polygon(
              points={{60,22},{56,18},{0,64},{-54,20},{-54,-40},{60,-40},{60,-46},{
                  -60,-46},{-60,24},{0,74},{60,22}},
              lineColor={127,0,0},
              smooth=Smooth.None,
              fillColor={127,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-46,6},{-46,-6},{-44,-8},{-24,4},{-24,16},{-26,18},{-46,6}},
              lineColor={127,0,0},
              smooth=Smooth.None,
              fillColor={127,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-46,-18},{-46,-30},{-44,-32},{-24,-20},{-24,-8},{-26,-6},{-46,
                  -18}},
              lineColor={127,0,0},
              smooth=Smooth.None,
              fillColor={127,0,0},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-44,-4},{-50,-8},{-50,-32},{-46,-36},{28,-36},{42,-26}},
              color={127,0,0},
              smooth=Smooth.None),
            Line(
              points={{-50,-32},{-44,-28}},
              color={127,0,0},
              smooth=Smooth.None),
            Line(
              points={{-24,14},{-20,16},{-20,-18},{-16,-22},{-16,-22},{40,-22}},
              color={127,0,0},
              smooth=Smooth.None),
            Line(
              points={{-24,-10},{-20,-8}},
              color={127,0,0},
              smooth=Smooth.None),
            Polygon(
              points={{40,-12},{40,-32},{50,-38},{58,-32},{58,-16},{54,-10},{48,-6},
                  {40,-12}},
              lineColor={127,0,0},
              smooth=Smooth.None,
              fillColor={127,0,0},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-100,-60},{100,-100}},
              lineColor={127,0,0},
              textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=false,
                       extent={{-100,-100},{100,100}}), graphics));
    end Building;

    model Feeder

      parameter Integer nLoads(min=1) "number of electric loads";
      parameter Boolean backbone=false;

      Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin
        plugBackbone if backbone "Electricity connection for the backbone grid"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin[nLoads]
        plugFeeder "Electricity connection for the buildings"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      annotation (Icon(graphics={
            Line(
              points={{30,36},{54,18},{100,4}},
              color={85,170,255},
              smooth=Smooth.Bezier),
            Line(
              points={{-22,36},{30,2},{100,0}},
              color={85,170,255},
              smooth=Smooth.Bezier),
            Polygon(
              points={{-32,40},{-32,34},{-4,34},{-4,-80},{4,-80},{4,34},{34,34},{34,
                  40},{4,40},{4,46},{-4,46},{-4,40},{-32,40}},
              lineColor={95,95,95},
              smooth=Smooth.None,
              fillPattern=FillPattern.Solid,
              fillColor={95,95,95}),
            Line(
              points={{-102,4},{-46,12},{-28,36}},
              color={85,170,255},
              smooth=Smooth.Bezier),
            Line(
              points={{-100,0},{-12,12},{30,36}},
              color={85,170,255},
              smooth=Smooth.Bezier)}), Diagram(graphics));

    end Feeder;

    package Examples
    extends Modelica.Icons.ExamplesPackage;
      model building
        extends BM_ABC.Interfaces.Building(
          redeclare IDEAS.Buildings.Examples.BaseClasses.structure building,
          redeclare IDEAS.VentilationSystems.None ventilationSystem,
          redeclare IDEAS.Occupants.Standards.None occupant,
          redeclare IDEAS.HeatingSystems.None heatingSystem,
          redeclare BaseClasses.CausalInhomeFeeder inHomeGrid);

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end building;
    end Examples;

    package BaseClasses

    extends Modelica.Icons.BasesPackage;

      model CausalInhomeFeeder
        "Causal inhome feeder model for a single phase grid connection"

        // Building characteristics  //////////////////////////////////////////////////////////////////////////

        parameter Modelica.SIunits.Length len=10
          "Cable length to district feeder";

        // Interfaces  ///////////////////////////////////////////////////////////////////////////////////////

        Modelica.Blocks.Interfaces.RealOutput VGrid
          annotation (Placement(transformation(extent={{96,30},{116,50}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
          nodeSingle(m=1)
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin
          pinSingle annotation (Placement(transformation(extent={{90,-10},{110,10}}),
              iconTransformation(extent={{90,-10},{110,10}})));

        // Components  ///////////////////////////////////////////////////////////////////////////////////////

        IDEAS.Electric.BaseClasses.AC.WattsLaw wattsLaw(numPha=1)
          annotation (Placement(transformation(extent={{20,-10},{40,10}})));

        IDEAS.Electric.Distribution.AC.BaseClasses.BranchLenTyp branch(len=len)
          "Cable to district feeder"
          annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        IDEAS.Electric.BaseClasses.AC.PotentialSensor voltageSensor
          "District feeder voltagesensor" annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={50,20})));

      protected
        Modelica.Electrical.QuasiStationary.SinglePhase.Sources.VoltageSource
          voltageSource(
          f=50,
          V=230,
          phi=0) "Steady building-side 230 V voltage source" annotation (Placement(
              transformation(
              extent={{-8,-8},{8,8}},
              rotation=270,
              origin={-80,-42})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground(pin(
              final reference))
          "Grounding for the building-side voltage source"
          annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPin_p plugToPin_p(
            m=1) "Plug-to-pin conversion" annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-80,-22})));

      algorithm
        wattsLaw.P := -Modelica.ComplexMath.real(plugToPin_p.plug_p.pin[1].v*
          Modelica.ComplexMath.conj(plugToPin_p.plug_p.pin[1].i));
        wattsLaw.Q := -Modelica.ComplexMath.imag(plugToPin_p.plug_p.pin[1].v*
          Modelica.ComplexMath.conj(plugToPin_p.plug_p.pin[1].i));

      equation
        connect(nodeSingle, plugToPin_p.plug_p) annotation (Line(
            points={{-100,0},{-80,0},{-80,-20},{-80,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource.pin_p, plugToPin_p.pin_p) annotation (Line(
            points={{-80,-34},{-80,-24}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(ground.pin, voltageSource.pin_n) annotation (Line(
            points={{-80,-60},{-80,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(wattsLaw.vi[1], voltageSensor.vi) annotation (Line(
            points={{40,0},{50,0},{50,10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSensor.VGrid, VGrid) annotation (Line(
            points={{50,30.4},{50,40},{106,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(wattsLaw.vi[1], branch.pin_p) annotation (Line(
            points={{40,0},{60,0}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(branch.pin_n, pinSingle) annotation (Line(
            points={{80,0},{100,0}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Rectangle(
                extent={{28,60},{70,20}},
                lineColor={85,170,255},
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-26,54},{-26,20},{-6,20},{-6,28},{4,28},{4,32},{-6,32},{-6,
                    44},{8,44},{8,50},{-6,50},{-6,54},{-26,54}},
                lineColor={85,170,255},
                smooth=Smooth.None,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-14,20},{-14,0},{-94,0}},
                color={85,170,255},
                smooth=Smooth.None),
              Rectangle(
                extent={{46,50},{50,42}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{56,34},{60,26}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{38,34},{42,26}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{48,20},{48,0},{96,0}},
                color={85,170,255},
                smooth=Smooth.None)}),
          Diagram(graphics),
          Documentation(info="<html>
<p>This gives an in home grid with single phase plugs and single phase grid connection</p>
</html>"));
      end CausalInhomeFeeder;

      model InhomeFeeder

        extends Modelica.Icons.ObsoleteModel;

        parameter Integer nHeatingLoads(min=1)
          "number of electric loads for the heating system";
        parameter Integer nVentilationLoads(min=1)
          "number of electric loads for the ventilation system";
        parameter Integer nOccupantLoads(min=1)
          "number of electric loads for the occupants";
        parameter Integer numberOfPhazes=1
          "The number of phazes connected in the home" annotation (choices(choice=1
              "Single phaze grid connection", choice=4
              "threephaze (4 line) grid connection"));

        inner outer IDEAS.SimInfoManager sim
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin
          plugFeeder[numberOfPhazes]
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug[
          nVentilationLoads] plugVentilationLoad
          "Electricity connection for the ventilaiton system"
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug[
          nHeatingLoads] plugHeatingLoad
          "Electricity connection for the heating system"
          annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug[
          nOccupantLoads] plugOccupantLoad
          "Electricity connection for the occupants"
          annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
        annotation (Icon(graphics), Diagram(graphics));

      end InhomeFeeder;

      model PartialSystem "General partial for electricity-based systems"

        outer IDEAS.SimInfoManager sim
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-200,80},{-180,100}})));

        // --- Electrical
        parameter Integer nLoads(min=0) = 1
          "Number of electric loads. If zero, all electric equations disappear.";
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
          plugLoad(m=1) if nLoads >= 1
          "Electricity connection to the Inhome feeder"
          annotation (Placement(transformation(extent={{190,-10},{210,10}})));
        IDEAS.Electric.BaseClasses.AC.WattsLawPlug wattsLawPlug(each numPha=1, final nLoads=
              nLoads) if nLoads >= 1
          annotation (Placement(transformation(extent={{170,-10},{190,10}})));

        // --- Sensor

        final parameter Integer nLoads_min=max(1, nLoads);
        Modelica.SIunits.Power[nLoads_min] P
          "Active power for each of the loads";
        Modelica.SIunits.Power[nLoads_min] Q
          "Passive power for each of the loads";
        Modelica.Blocks.Sources.RealExpression[nLoads_min] P_val(y=P)
          annotation (Placement(transformation(extent={{144,-5},{164,15}})));
        Modelica.Blocks.Sources.RealExpression[nLoads_min] Q_val(y=Q)
          annotation (Placement(transformation(extent={{144,-20},{164,0}})));
      equation
        if nLoads >= 1 then
          connect(wattsLawPlug.vi, plugLoad) annotation (Line(
              points={{190,0},{200,0}},
              color={85,170,255},
              smooth=Smooth.None));
        end if;
        connect(P_val.y, wattsLawPlug.P) annotation (Line(
            points={{165,5},{174,5}},
            color={0,0,127},
            smooth=Smooth.None));

        connect(Q_val.y, wattsLawPlug.Q) annotation (Line(
            points={{165,-10},{170,-10},{170,1},{173,1}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,
                  100}}), graphics),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,
                  100}}),     graphics),
          Documentation(info="<html>
<p><b>Description</b> </p>
<p>Interface model for a complete multi-zone heating system (with our without domestic hot water and solar system).</p>
<p>This model defines the ports used to link a heating system with a building, and the basic parameters that most heating systems will need to have. The model is modular as a function of the number of zones <i>nZones. </i></p>
<p>Two sets of heatPorts are defined:</p>
<p><ol>
<li><i>heatPortCon[nZones]</i> and <i>heatPortRad[nZones]</i> for convective respectively radiative heat transfer to the building. </li>
<li><i>heatPortEmb[nZones]</i> for heat transfer to TABS elements in the building. </li>
</ol></p>
<p>The model also defines <i>TSensor[nZones]</i> and <i>TSet[nZones]</i> for the control, and a nominal power <i>QNom[nZones].</i></p>
<p>There is also an input for the DHW flow rate, <i>mDHW60C</i>, but this can be unconnected if the system only includes heating and no DHW.</p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>See the different extensions of this model in <a href=\"modelica://IDEAS.Thermal.HeatingSystems\">IDEAS.Thermal.HeatingSystems</a></li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://IDEAS.Interfaces.BaseClasses.Structure\">structure</a>. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> and <i>plugLoad. </i></li>
<li>Connect <i>plugLoad </i> to an inhome grid.  A<a href=\"modelica://IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder\"> dummy inhome grid like this</a> has to be used if no inhome grid is to be modelled. </li>
<li>Set all parameters that are required, depending on which implementation of this interface is used. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>No validation performed.</p>
<p><h4>Example </h4></p>
<p>See the <a href=\"modelica://IDEAS.Thermal.HeatingSystems.Examples\">heating system examples</a>. </p>
</html>"));
      end PartialSystem;

      model Structure "Partial model for building structure models"

        outer IDEAS.SimInfoManager sim
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{244,-112},{264,-92}})));

        replaceable package Medium = Modelica.Media.Air.SimpleAir
          constrainedby Modelica.Media.Interfaces.PartialMedium
          "Medium in the component"
            annotation (choicesAllMatching = true);

        // Building characteristics  //////////////////////////////////////////////////////////////////////////

        parameter Integer nZones(min=1)=12
          "Number of conditioned thermal zones in the building";
        parameter Integer nEmb(min=0)=0
          "Number of embedded systems in the building";
        parameter Modelica.SIunits.Area ATrans=100
          "Transmission heat loss area of the residential unit";
        parameter Modelica.SIunits.Area[nZones] AZones = ones(nZones)*57.1298
          "Conditioned floor area of the zones";
        parameter Modelica.SIunits.Volume[nZones] VZones = ones(nZones)*308.5
          "Conditioned volume of the zones based on external dimensions";
        final parameter Modelica.SIunits.Length C=sum(VZones)/ATrans
          "Building compactness";

        parameter Modelica.SIunits.Temperature[nZones] T_start
          "Operative zonal start temperatures";

        parameter Modelica.SIunits.Power[ nZones] Q_design=zeros(nZones)
          "Design heat loss of zones";//must be filled in in the Building interface, e.g.: QDesign={building.zone1.Q_design,building.zone2.Q_design}

        // Interfaces  ///////////////////////////////////////////////////////////////////////////////////////

        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nZones] heatPortCon
          "Internal zone nodes for convective heat gains" annotation (Placement(
              transformation(extent={{260,26},{280,46}}), iconTransformation(extent={{260,26},
                  {280,46}})));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nZones] heatPortRad
          "Internal zones node for radiative heat gains" annotation (Placement(
              transformation(extent={{260,-14},{280,6}}),   iconTransformation(extent={{260,-14},
                  {280,6}})));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b[nEmb] heatPortEmb
          "Construction nodes for heat gains by embedded layers" annotation (
            Placement(transformation(extent={{260,66},{280,86}}), iconTransformation(
                extent={{260,66},{280,86}})));
        Modelica.Blocks.Interfaces.RealOutput[nZones] TSensor(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0)
          "Sensor temperature of the zones"
          annotation (Placement(transformation(extent={{266,-54},{286,-34}})));
        IDEAS.Fluid.Interfaces.FlowPort_b[nZones] flowPort_Out(redeclare
            package Medium =
                     Medium)
          annotation (Placement(transformation(extent={{-30,118},{-10,138}})));
        IDEAS.Fluid.Interfaces.FlowPort_a[nZones] flowPort_In(redeclare package
            Medium = Medium)
          annotation (Placement(transformation(extent={{10,118},{30,138}})));
        outer Modelica.Fluid.System system
        annotation (Placement(transformation(extent={{214,-112},{234,-92}})));
        Blocks.Apartment_A12 apartment_A12_1
          annotation (Placement(transformation(extent={{-208,8},{-136,50}})));
        Blocks.Apartment_A02 apartment_A02_1 annotation (Placement(
              transformation(extent={{-212,-58},{-136,-16}})));
        Blocks.Apartment_A11 apartment_A11_1
          annotation (Placement(transformation(extent={{-156,8},{-80,50}})));
        Blocks.Apartment_A01 apartment_A01_1 annotation (Placement(
              transformation(extent={{-160,-60},{-76,-16}})));
        Blocks.Apartment_B12 apartment_B12_1
          annotation (Placement(transformation(extent={{-80,10},{-2,50}})));
        Blocks.Apartment_B02 apartment_B02_1
          annotation (Placement(transformation(extent={{-80,-58},{-2,-18}})));
        Blocks.Apartment_B11 apartment_B11_1
          annotation (Placement(transformation(extent={{-16,10},{56,50}})));
        Blocks.Apartment_B01 apartment_B01_1
          annotation (Placement(transformation(extent={{-20,-58},{58,-16}})));
        Blocks.Apartment_C12 apartment_C12_1
          annotation (Placement(transformation(extent={{52,8},{128,48}})));
        Blocks.Apartment_C02 apartment_C02_1
          annotation (Placement(transformation(extent={{54,-58},{128,-18}})));
        Blocks.Apartment_C11 apartment_C11_1
          annotation (Placement(transformation(extent={{118,8},{190,46}})));
        Blocks.Apartment_C01 apartment_C01_1
          annotation (Placement(transformation(extent={{116,-58},{194,-20}})));
        Blocks.Basement_A basement_A
          annotation (Placement(transformation(extent={{-186,-110},{-100,-78}})));
        Blocks.Basement_B basement_B
          annotation (Placement(transformation(extent={{-50,-110},{34,-78}})));
        Blocks.Basement_C basement_C
          annotation (Placement(transformation(extent={{84,-110},{170,-78}})));
        Blocks.Attic_A attic_A
          annotation (Placement(transformation(extent={{-174,70},{-110,96}})));
        Blocks.Attic_B attic_B
          annotation (Placement(transformation(extent={{-42,70},{26,96}})));
        Blocks.Attic_C attic_C
          annotation (Placement(transformation(extent={{88,68},{160,96}})));
      equation

        //Ventilation//

        //A Block//

        apartment_A12_1.flowPort_Out1= flowPort_Out[1];
        apartment_A12_1.flowPort_In1= flowPort_In[1];

        apartment_A11_1.flowPort_Out1= flowPort_Out[2];
        apartment_A11_1.flowPort_In1= flowPort_In[2];

        apartment_A02_1.flowPort_Out1= flowPort_Out[3];
        apartment_A02_1.flowPort_In1= flowPort_In[3];

        apartment_A01_1.flowPort_Out1= flowPort_Out[4];
        apartment_A01_1.flowPort_In1= flowPort_In[4];

        //B Block//

        apartment_B12_1.flowPort_Out1= flowPort_Out[5];
        apartment_B12_1.flowPort_In1= flowPort_In[5];

        apartment_B11_1.flowPort_Out1= flowPort_Out[6];
        apartment_B11_1.flowPort_In1= flowPort_In[6];

        apartment_B02_1.flowPort_Out1= flowPort_Out[7];
        apartment_B02_1.flowPort_In1= flowPort_In[7];

        apartment_B01_1.flowPort_Out1= flowPort_Out[8];
        apartment_B01_1.flowPort_In1= flowPort_In[8];

        //C Block//

        apartment_C12_1.flowPort_Out1= flowPort_Out[9];
        apartment_C12_1.flowPort_In1= flowPort_In[9];

        apartment_C11_1.flowPort_Out1= flowPort_Out[10];
        apartment_C11_1.flowPort_In1= flowPort_In[10];

        apartment_C02_1.flowPort_Out1= flowPort_Out[11];
        apartment_C02_1.flowPort_In1= flowPort_In[11];

        apartment_C01_1.flowPort_Out1= flowPort_Out[12];
        apartment_C01_1.flowPort_In1= flowPort_In[12];

        //Temperature Sensors

        //A block//

        apartment_A12_1.TSensor1= TSensor[1];
        apartment_A11_1.TSensor1= TSensor[2];
        apartment_A02_1.TSensor1= TSensor[3];
        apartment_A01_1.TSensor1= TSensor[4];

        //B block//

        apartment_B12_1.TSensor1= TSensor[5];
        apartment_B11_1.TSensor1= TSensor[6];
        apartment_B02_1.TSensor1= TSensor[7];
        apartment_B01_1.TSensor1= TSensor[8];

        //C block//

        apartment_C12_1.TSensor1= TSensor[9];
        apartment_C11_1.TSensor1= TSensor[10];
        apartment_C02_1.TSensor1= TSensor[11];
        apartment_C01_1.TSensor1= TSensor[12];

        //Radiative Heat Gains//

        //A block//

        apartment_A12_1.gainRad1= TSensor[1];
        apartment_A11_1.gainRad1= TSensor[2];
        apartment_A02_1.gainRad1= TSensor[3];
        apartment_A01_1.gainRad1= TSensor[4];

        //B block//

        apartment_B12_1.gainRad1= TSensor[5];
        apartment_B11_1.gainRad1= TSensor[6];
        apartment_B02_1.gainRad1= TSensor[7];
        apartment_B01_1.gainRad1= TSensor[8];

        //C block//

        apartment_C12_1.gainRad1= TSensor[9];
        apartment_C11_1.gainRad1= TSensor[10];
        apartment_C02_1.gainRad1= TSensor[11];
        apartment_C01_1.gainRad1= TSensor[12];

        //Convective Heat Gains//

        //A block//

        apartment_A12_1.gainCon1= TSensor[1];
        apartment_A11_1.gainCon1= TSensor[2];
        apartment_A02_1.gainCon1= TSensor[3];
        apartment_A01_1.gainCon1= TSensor[4];

        //B block//

        apartment_B12_1.gainCon1= TSensor[5];
        apartment_B11_1.gainCon1= TSensor[6];
        apartment_B02_1.gainCon1= TSensor[7];
        apartment_B01_1.gainCon1= TSensor[8];

        //C block//

        apartment_C12_1.gainCon1= TSensor[9];
        apartment_C11_1.gainCon1= TSensor[10];
        apartment_C02_1.gainCon1= TSensor[11];
        apartment_C01_1.gainCon1= TSensor[12];

        //Ceiling-Floor Connections//

        //Attic to Upper Apt//

        apartment_A12_1.zoneBus[3]= attic_A.zoneBus[1];
        apartment_B12_1.zoneBus[3]= attic_B.zoneBus[1];
        apartment_C12_1.zoneBus[3]= attic_C.zoneBus[1];

        apartment_A11_1.zoneBus[3]= attic_A.zoneBus[2];
        apartment_B11_1.zoneBus[3]= attic_B.zoneBus[2];
        apartment_C11_1.zoneBus[3]= attic_C.zoneBus[2];

        //Upper Apt to Lower Apt//

        //A Block//

        apartment_A12_1.zoneBus[2]= apartment_A02_1.zoneBus[3];
        apartment_A11_1.zoneBus[2]= apartment_A01_1.zoneBus[3];

        //B Block//

        apartment_B12_1.zoneBus[2]= apartment_B02_1.zoneBus[3];
        apartment_B11_1.zoneBus[2]= apartment_B01_1.zoneBus[3];

        //C Block//

        apartment_C12_1.zoneBus[2]= apartment_C02_1.zoneBus[3];
        apartment_C11_1.zoneBus[2]= apartment_C01_1.zoneBus[3];

        //Lower Apt to Basement//

        apartment_A02_1.zoneBus[2]= basement_A.zoneBus[1];
        apartment_B02_1.zoneBus[2]= basement_B.zoneBus[1];
        apartment_C02_1.zoneBus[2]= basement_C.zoneBus[1];

        apartment_A01_1.zoneBus[2]= basement_A.zoneBus[2];
        apartment_B01_1.zoneBus[2]= basement_B.zoneBus[2];
        apartment_C01_1.zoneBus[2]= basement_C.zoneBus[2];

        //Interconnection between Apartments in the same block//

        apartment_A12_1.zoneBus[1]= apartment_A11_1.zoneBus[4];
        apartment_B12_1.zoneBus[1]= apartment_B11_1.zoneBus[4];
        apartment_C12_1.zoneBus[1]= apartment_C11_1.zoneBus[4];

        apartment_A02_1.zoneBus[1]= apartment_A01_1.zoneBus[4];
        apartment_B02_1.zoneBus[1]= apartment_B01_1.zoneBus[4];
        apartment_C02_1.zoneBus[1]= apartment_C01_1.zoneBus[4];

        //Interconnection between Apartments in different blocks//

        //A and B//

        apartment_A11_1.zoneBus[1]= apartment_B12_1.zoneBus[4];
        apartment_A01_1.zoneBus[1]= apartment_B02_1.zoneBus[4];

        //B and C//
        apartment_B11_1.zoneBus[1]= apartment_C12_1.zoneBus[4];
        apartment_B01_1.zoneBus[1]= apartment_C02_1.zoneBus[4];
           annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,-150},
                  {300,150}}), graphics={
              Rectangle(
                extent={{-150,100},{150,-100}},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                lineColor={191,0,0}),
              Line(
                points={{60,8},{0,60},{-60,10},{-60,-60},{60,-60}},
                color={127,0,0},
                smooth=Smooth.None),
              Polygon(
                points={{60,8},{56,4},{0,50},{-52,8},{-52,-52},{60,-52},{60,-60},{-60,
                    -60},{-60,10},{0,60},{60,8}},
                lineColor={95,95,95},
                smooth=Smooth.None,
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-300,-150},{300,150}}), graphics));
      end Structure;

      model CausalInhomeFeeder_StandAlone
        "Causal inhome feeder model for a single phase grid connection"

        // Building characteristics  //////////////////////////////////////////////////////////////////////////

        parameter Modelica.SIunits.Length len=10
          "Cable length to district feeder";

        // Interfaces  ///////////////////////////////////////////////////////////////////////////////////////

        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
          nodeSingle(m=1)
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin
          pinSingle annotation (Placement(transformation(extent={{90,-10},{110,10}}),
              iconTransformation(extent={{90,-10},{110,10}})));

        // Components  ///////////////////////////////////////////////////////////////////////////////////////

      protected
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPin_p plugToPin_p(
            m=1) "Plug-to-pin conversion" annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-80,0})));

      equation
        connect(nodeSingle, plugToPin_p.plug_p) annotation (Line(
            points={{-100,0},{-82,0}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(plugToPin_p.pin_p, pinSingle) annotation (Line(
            points={{-78,0},{100,0}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Rectangle(
                extent={{28,60},{70,20}},
                lineColor={85,170,255},
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-26,54},{-26,20},{-6,20},{-6,28},{4,28},{4,32},{-6,32},{-6,
                    44},{8,44},{8,50},{-6,50},{-6,54},{-26,54}},
                lineColor={85,170,255},
                smooth=Smooth.None,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-14,20},{-14,0},{-94,0}},
                color={85,170,255},
                smooth=Smooth.None),
              Rectangle(
                extent={{46,50},{50,42}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{56,34},{60,26}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{38,34},{42,26}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{48,20},{48,0},{96,0}},
                color={85,170,255},
                smooth=Smooth.None)}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}),
                  graphics),
          Documentation(info="<html>
<p>This gives an in home grid with single phase plugs and single phase grid connection</p>
</html>"));
      end CausalInhomeFeeder_StandAlone;

      model VentilationSystem
        "Ventilation with constant air flow at constant temperature and no power calculations"
        extends IDEAS.Interfaces.BaseClasses.VentilationSystem(nLoads=1);
        parameter Modelica.SIunits.MassFlowRate m_flow[nZones] = zeros(nZones)
          "Ventilation mass flow rate per zones";
        parameter Modelica.SIunits.Temperature TSet[nZones] = 22*.ones(nZones) .+ 273.15
          "Ventilation set point temperature per zone";

        IDEAS.Fluid.Sources.MassFlowSource_T sou[nZones](
          each use_m_flow_in=true,
          each final nPorts=1,
          redeclare each package Medium = Medium,
          each use_T_in=true) "Source"
          annotation (Placement(transformation(extent={{-162,12},{-182,32}})));
        IDEAS.Fluid.Sources.FixedBoundary sink[nZones](each final nPorts=1,
            redeclare each package Medium = Medium)
          annotation (Placement(transformation(extent={{-162,-28},{-182,-8}})));
        Modelica.Blocks.Sources.Constant m_flow_val[nZones](k=0.0563)
          annotation (Placement(transformation(extent={{7,-7},{-7,7}},
              rotation=180,
              origin={-179,53})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,-82})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,-82})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,-82})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp1(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,-66})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim1
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,-66})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin1 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,-66})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp2(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,-52})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim2
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,-52})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin2 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,-52})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp3(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,-38})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim3
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,-38})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin3 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,-38})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp4(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,-24})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim4
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,-24})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin4 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,-24})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp5(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,-10})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim5
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,-10})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin5 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,-10})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp6(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,4})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim6
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,4})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin6 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,4})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp7(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,18})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim7
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,18})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin7 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,18})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp8(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,32})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim8
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,32})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin8 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,32})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp9(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,46})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime simTim9
          annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,46})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin9 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,46})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp10(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,60})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime
          simTim10 annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,60})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin10 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,60})));
        Modelica.Blocks.Tables.CombiTable1D OutsideTemp11(
          tableOnFile=true,
          tableName="data",
          fileName=Modelica.Utilities.Files.loadResource("modelica://IDEAS") + "/Inputs/",
          columns={2}) annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-76,74})));

        IDEAS.BoundaryConditions.WeatherData.BaseClasses.SimulationTime
          simTim11 annotation (Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-60,74})));
        Modelica.Thermal.HeatTransfer.Celsius.ToKelvin toKelvin11 annotation (
            Placement(transformation(
              extent={{-4,-4},{4,4}},
              rotation=180,
              origin={-94,74})));
      equation
        P[1:nLoads_min] = zeros(nLoads_min);
        Q[1:nLoads_min] = zeros(nLoads_min);

        connect(flowPort_Out[:], sou[:].ports[1]);
        connect(flowPort_In[:], sink[:].ports[1]);

        connect(sou.m_flow_in, m_flow_val.y) annotation (Line(
            points={{-162,30},{-154,30},{-154,53},{-171.3,53}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp.y[1], toKelvin.Celsius) annotation (Line(
            points={{-80.4,-82},{-89.2,-82}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim.y, OutsideTemp.u[1]) annotation (Line(
            points={{-64.4,-82},{-71.2,-82}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp1.y[1], toKelvin1.Celsius) annotation (Line(
            points={{-80.4,-66},{-89.2,-66}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim1.y, OutsideTemp1.u[1]) annotation (Line(
            points={{-64.4,-66},{-71.2,-66}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp2.y[1], toKelvin2.Celsius) annotation (Line(
            points={{-80.4,-52},{-89.2,-52}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim2.y, OutsideTemp2.u[1]) annotation (Line(
            points={{-64.4,-52},{-71.2,-52}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp3.y[1], toKelvin3.Celsius) annotation (Line(
            points={{-80.4,-38},{-89.2,-38}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim3.y, OutsideTemp3.u[1]) annotation (Line(
            points={{-64.4,-38},{-71.2,-38}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp4.y[1], toKelvin4.Celsius) annotation (Line(
            points={{-80.4,-24},{-89.2,-24}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim4.y, OutsideTemp4.u[1]) annotation (Line(
            points={{-64.4,-24},{-71.2,-24}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp5.y[1], toKelvin5.Celsius) annotation (Line(
            points={{-80.4,-10},{-89.2,-10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim5.y, OutsideTemp5.u[1]) annotation (Line(
            points={{-64.4,-10},{-71.2,-10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp6.y[1], toKelvin6.Celsius) annotation (Line(
            points={{-80.4,4},{-89.2,4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim6.y, OutsideTemp6.u[1]) annotation (Line(
            points={{-64.4,4},{-71.2,4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp7.y[1], toKelvin7.Celsius) annotation (Line(
            points={{-80.4,18},{-89.2,18}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim7.y, OutsideTemp7.u[1]) annotation (Line(
            points={{-64.4,18},{-71.2,18}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp8.y[1], toKelvin8.Celsius) annotation (Line(
            points={{-80.4,32},{-89.2,32}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim8.y, OutsideTemp8.u[1]) annotation (Line(
            points={{-64.4,32},{-71.2,32}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp9.y[1], toKelvin9.Celsius) annotation (Line(
            points={{-80.4,46},{-89.2,46}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim9.y, OutsideTemp9.u[1]) annotation (Line(
            points={{-64.4,46},{-71.2,46}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp10.y[1], toKelvin10.Celsius) annotation (Line(
            points={{-80.4,60},{-89.2,60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim10.y, OutsideTemp10.u[1]) annotation (Line(
            points={{-64.4,60},{-71.2,60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(OutsideTemp11.y[1], toKelvin11.Celsius) annotation (Line(
            points={{-80.4,74},{-89.2,74}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(simTim11.y, OutsideTemp11.u[1]) annotation (Line(
            points={{-64.4,74},{-71.2,74}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin.Kelvin, sou[1].T_in) annotation (Line(
            points={{-98.4,-82},{-128,-82},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin1.Kelvin, sou[2].T_in) annotation (Line(
            points={{-98.4,-66},{-128,-66},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin2.Kelvin, sou[3].T_in) annotation (Line(
            points={{-98.4,-52},{-128,-52},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin3.Kelvin, sou[4].T_in) annotation (Line(
            points={{-98.4,-38},{-128,-38},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin4.Kelvin, sou[5].T_in) annotation (Line(
            points={{-98.4,-24},{-128,-24},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin5.Kelvin, sou[6].T_in) annotation (Line(
            points={{-98.4,-10},{-128,-10},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin6.Kelvin, sou[7].T_in) annotation (Line(
            points={{-98.4,4},{-128,4},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin7.Kelvin, sou[8].T_in) annotation (Line(
            points={{-98.4,18},{-128,18},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin8.Kelvin, sou[9].T_in) annotation (Line(
            points={{-98.4,32},{-128,32},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin9.Kelvin, sou[10].T_in) annotation (Line(
            points={{-98.4,46},{-128,46},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin10.Kelvin, sou[11].T_in) annotation (Line(
            points={{-98.4,60},{-128,60},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(toKelvin11.Kelvin, sou[12].T_in) annotation (Line(
            points={{-98.4,74},{-128,74},{-128,26},{-160,26}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                  -100},{200,100}}), graphics), Icon(coordinateSystem(extent={{-200,
                  -100},{200,100}})));
      end VentilationSystem;

      model Occupant "Occupant ISO13790 Standard"
        extends IDEAS.Interfaces.BaseClasses.Occupant(nZones=1, nLoads=1);

        parameter Modelica.SIunits.Area[nZones] AFloor=ones(nZones)*57.1298
          "Floor area of different zones";

      protected
        final parameter Modelica.SIunits.Time interval=3600 "Time interval";
        final parameter Modelica.SIunits.Time period=86400/interval
          "Number of intervals per repetition";
        final parameter Real[3] QDay(unit="W/m2") = {8,20,2}
          "Specific power for dayzone";
        Integer t "Time interval";

      algorithm
        when sample(0, interval) then
          t := if pre(t) + 1 <= period then pre(t) + 1 else 1;
        end when;

      equation
        mDHW60C = 0;
        heatPortRad.Q_flow = heatPortCon.Q_flow;
        P = {heatPortCon[1].Q_flow + heatPortRad[1].Q_flow};
        Q = {0};

        if noEvent(t <= 7 or t >= 23) then
          heatPortCon.Q_flow = -AFloor*QDay[3]*0.5;
          TSet = ones(nZones)*(18 + 273.15);
        elseif noEvent(t > 7 and t <= 17) then
          heatPortCon.Q_flow = -AFloor*QDay[1]*0.5;
          TSet = ones(nZones)*(16 + 273.15);
        else
          heatPortCon.Q_flow = -AFloor*QDay[2]*0.5;
          TSet = ones(nZones)*(21 + 273.15);
        end if;

        annotation (Diagram(graphics));
      end Occupant;

      model HeatingSystem "Ideal heating, no DHW, with radiators"
        extends IDEAS.HeatingSystems.Interfaces.Partial_IdealHeating;
        extends IDEAS.Interfaces.BaseClasses.HeatingSystem(
          final isHea = true,
          final isCoo = false,
            nConvPorts = nZones,
            nRadPorts = nZones,
            nTemSen = nZones,
            nEmbPorts=0,
          nLoads=1);

      equation
         for i in 1:nZones loop
           if noEvent((TSet[i] - TSensor[i]) > 0) then
             QHeatZone[i] = IDEAS.Utilities.Math.Functions.smoothMin(x1=C[i]*(TSet[i] - TSensor[i])/t, x2=QNom[i],deltaX=1);
           else
             QHeatZone[i] = 0;
           end if;
           heatPortRad[i].Q_flow = -fractionRad[i]*QHeatZone[i];
           heatPortCon[i].Q_flow = -(1 - fractionRad[i])*QHeatZone[i];
         end for;
        QHeaSys = sum(QHeatZone);

        P[1] = QHeaSys/COP;
        Q[1] = 0;
        annotation (Documentation(info="<html>
<p><b>Description</b> </p>
<p>Ideal heating (no hydraulics) but with limited power <i>QNom</i> per zone. There are no radiators. This model assumes a thermal inertia of each zone and computes the heat flux that would be required to heat up the zone to the set point within a time <i>t</i>. This heat flux is limited to <i>QNom</i> and splitted in a radiative and a convective part which are imposed on the heatPorts <i>heatPortRad</i> and <i>heatPortCon</i> respectively. A COP can be passed in order to compute the electricity consumption of the heating.</p>
<p><u>Note</u>: the responsiveness of the system is influenced by the time constant <i>t</i>.  For small values of<i> t</i>, this system is close to ideal, but for larger values, there may still be deviations between the zone temperature and it&apos;s set point. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>No inertia; responsiveness modelled by time constant <i>t</i> for reaching the temperature set point. </li>
<li>Limited output power according to <i>QNom[nZones]</i></li>
<li>Heat emitted through <i>heatPortRad</i> and <i>heatPortCon</i> </li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://IDEAS.Interfaces.BaseClasses.Structure\">structure</a>. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Connect <i>plugLoad </i>to an inhome grid. A<a href=\"modelica://IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder\"> dummy inhome grid like this</a> has to be used if no inhome grid is to be modelled. </li>
<li>Set all parameters that are required. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>No validation performed.</p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in<a href=\"modelica://IDEAS.Thermal.HeatingSystems.Examples.IdealRadiatorHeating\"> IDEAS.Thermal.HeatingSystems.Examples.IdealRadiatorHeating</a>.</p>
</html>",       revisions="<html>
<p><ul>
<li>2013 June, Roel De Coninck: reworking interface and documentation</li>
<li>2011, Roel De Coninck: first version</li>
</ul></p>
</html>"),       Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                  -100},{200,100}}),
                               graphics),
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
              graphics));
      end HeatingSystem;
    end BaseClasses;
  end Interfaces;

 package Blocks

   model Apartment_A12

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        AWall=18.57025) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
        A=7.05275,
        inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
        inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
        frac=0.1385,
        inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall WestWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.EastWest_InsulatedWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       AWall=65.016,
        inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=65.016,
        inc=1.5707963267949) "Internal Wall between the Apartemnts"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-91,40})));
     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=38.61,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=46.737,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone A12(          V=308.5, nSurf=14)
       annotation (Placement(transformation(extent={{2,-32},{36,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall Floor(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Floor"
                                          annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.InternalWall CeilingRoof(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.CeilingRoof constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Ceiling,
       AWall=57.1298,
        inc=IDEAS.Constants.Ceiling) "Ceiling under the Roof"
                                                             annotation (Placement(
           transformation(
           extent={{-5,10},{5,-10}},
           rotation=270,
           origin={89,44})));
     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[3]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{54,10},{74,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{78,-30},{92,-16}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{74,-52},{90,-36}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{74,-78},{90,-62}})));

   equation
     connect(NorthWall.propsBus_a,A12. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.62857},{2,-6.62857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,A12. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.08571},{2,-7.08571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,A12. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.54286},{2,-7.54286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,A12. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-8},{2,-8}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall.propsBus_a,A12. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.45714},{2,-8.45714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,A12. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.91429},{2,-8.91429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,A12. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.37143},{2,-9.37143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,A12. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.82857},{2,-9.82857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,A12. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.2857},{2,-10.2857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,A12. propsBus[10]) annotation (Line(
         points={{183,-21},{184.5,-21},{184.5,-10.7429},{2,-10.7429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,A12. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-11.2},{2,-11.2}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_b,A12. propsBus[12]) annotation (Line(
         points={{-95,35},{-16.5,35},{-16.5,-11.6571},{2,-11.6571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_b,A12. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-12.1143},{2,-12.1143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(CeilingRoof.propsBus_a,A12. propsBus[14]) annotation (Line(
         points={{85,39},{-7.5,39},{-7.5,-12.5714},{2,-12.5714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(A12.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{15.6,0},{16,0},{16,20},{64,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A12.flowPort_In, flowPort_In1) annotation (Line(
         points={{22.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A12.TSensor, TSensor1) annotation (Line(
         points={{37.02,-16},{58,-16},{58,-23},{85,-23}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(A12.gainCon, gainCon1) annotation (Line(
         points={{36,-20.8},{52,-20.8},{52,-44},{82,-44}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(A12.gainRad, gainRad1) annotation (Line(
         points={{36,-25.6},{36,-70},{82,-70}},
         color={191,0,0},
         smooth=Smooth.None));
      connect(CeilingRoof.propsBus_b, zoneBus[3]) annotation (Line(
          points={{85,49},{85.5,49},{85.5,81.3333},{-2,81.3333}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(InternalWall1.propsBus_a, zoneBus[1]) annotation (Line(
          points={{-95,45},{-49.5,45},{-49.5,54.6667},{-2,54.6667}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(Floor.propsBus_a, zoneBus[2]) annotation (Line(
          points={{49,71},{27.5,71},{27.5,68},{-2,68}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="A12")}));
   end Apartment_A12;

   model Apartment_A02

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        AWall=17.813) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
        A=7.81,
        inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
        inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
        frac=0.1385,
        inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall WestWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.EastWest_InsulatedWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       AWall=65.016,
        inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=65.016,
        inc=1.5707963267949) "Internal Wall between the Apartemnts"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-91,40})));
     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=38.61,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=36.342,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone A02(          V=308.5, nSurf=14)
       annotation (Placement(transformation(extent={{-4,-30},{30,2}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall FloorAboveBasement(
        insulationThickness=0,
        redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor,
        redeclare BM_ABC.Data.Constructions.FloorAboveBasement constructionType)
        "Floor above the Basement" annotation (Placement(transformation(
            extent={{-5,-10},{5,10}},
            rotation=90,
            origin={53,66})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[3]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));

     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          = IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
            IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{74,-32},{88,-18}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{68,-56},{84,-40}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{68,-80},{82,-66}})));
   equation
     connect(NorthWall.propsBus_a,A02. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-4.62857},{-4,-4.62857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,A02. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-5.08571},{-4,-5.08571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,A02. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-5.54286},{-4,-5.54286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,A02. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-6},{-4,-6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall.propsBus_a,A02. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-6.45714},{-4,-6.45714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,A02. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-6.91429},{-4,-6.91429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,A02. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-7.37143},{-4,-7.37143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,A02. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-7.82857},{-4,-7.82857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,A02. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-8.28571},{-4,-8.28571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,A02. propsBus[10]) annotation (Line(
         points={{183,-21},{178.5,-21},{178.5,-8.74286},{-4,-8.74286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,A02. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-9.2},{-4,-9.2}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_b,A02. propsBus[12]) annotation (Line(
         points={{-95,35},{-16.5,35},{-16.5,-9.65714},{-4,-9.65714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_a, zoneBus[1]) annotation (Line(
         points={{-95,45},{-94.5,45},{-94.5,54.6667},{-2,54.6667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(FloorAboveBasement.propsBus_b, A02.propsBus[13]) annotation (Line(
          points={{49,61},{-11.5,61},{-11.5,-10.1143},{-4,-10.1143}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(FloorAboveBasement.propsBus_a, zoneBus[2]) annotation (Line(
          points={{49,71},{47.5,71},{47.5,68},{-2,68}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(A02.propsBus[14], zoneBus[3]) annotation (Line(
          points={{-4,-10.5714},{-4,81.3333},{-2,81.3333}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
     connect(A02.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{9.6,2},{34,2},{34,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A02.flowPort_In, flowPort_In1) annotation (Line(
         points={{16.4,2},{100,2},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A02.TSensor, TSensor1) annotation (Line(
         points={{31.02,-14},{54,-14},{54,-25},{81,-25}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(A02.gainCon, gainCon1) annotation (Line(
         points={{30,-18.8},{52,-18.8},{52,-48},{76,-48}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(A02.gainRad, gainRad1) annotation (Line(
         points={{30,-23.6},{52,-23.6},{52,-73},{75,-73}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="A02")}));
   end Apartment_A02;

   model Apartment_A11

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=18.57025) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=7.05275,
       inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
        inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
       frac=0.1385,
       inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        redeclare BM_ABC.Data.Constructions.EastWest_NonInsulatedWall
          constructionType,
        AWall=21.6,
        azi=IDEAS.Constants.East,
       inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=37.935,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=35.829,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone A11(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall Floor(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Floor" annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.InternalWall CeilingRoof(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.CeilingRoof constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Ceiling,
       AWall=57.1298,
       inc=IDEAS.Constants.Ceiling) "Ceiling under the Roof" annotation (Placement(
           transformation(
           extent={{-5,10},{5,-10}},
           rotation=270,
           origin={89,44})));
     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.InternalWall InternalWall4(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=43.416,
       inc=1.5707963267949) "Internal Wall between the Apartments of A and B"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-43,44})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          = IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{48,8},{68,28}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
            IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{74,-30},{88,-16}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{70,-54},{84,-40}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{70,-80},{84,-66}})));
   equation
     connect(NorthWall.propsBus_a,A11. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,A11. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,A11. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,A11. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,A11. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,A11. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,A11. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,A11. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,A11. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,A11. propsBus[10]) annotation (Line(
         points={{183,-21},{184.5,-21},{184.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,A11. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_b,A11. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(CeilingRoof.propsBus_a,A11. propsBus[14]) annotation (Line(
         points={{85,39},{-7.5,39},{-7.5,-12.16},{-1.77636e-015,-12.16}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{49.5,71},{49.5,63},{-2,63}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(CeilingRoof.propsBus_b, zoneBus[3]) annotation (Line(
         points={{85,49},{77.5,49},{77.5,73},{-2,73}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(InternalWall4.propsBus_a, zoneBus[1]) annotation (Line(
          points={{-47,49},{-46.5,49},{-46.5,53},{-2,53}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(InternalWall4.propsBus_b, A11.propsBus[12]) annotation (Line(
          points={{-47,39},{-20.5,39},{-20.5,-11.3067},{-1.77636e-015,-11.3067}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

     connect(A11.propsBus[15], zoneBus[4]) annotation (Line(
         points={{0,-12.5867},{0,83},{-2,83}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(A11.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{34,0},{34,20},{58,20},{58,18}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A11.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A11.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{54,-16},{54,-23},{81,-23}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(A11.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{54,-20.8},{54,-47},{77,-47}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(A11.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{34,-75.8},{77,-75.8},{77,-73}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
             textString="A11")}));
   end Apartment_A11;

   model Apartment_A01

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=17.813) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=7.81,
       inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
       frac=0.1385,
       inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        redeclare BM_ABC.Data.Constructions.EastWest_NonInsulatedWall
          constructionType,
        AWall=21.6,
        azi=IDEAS.Constants.East,
        inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=38.61,
       inc=1.5707963267949) "Internal Wall Small Brick" annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
       AWall=36.342,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone A01(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall FloorAboveBasement(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.FloorAboveBasement constructionType)
        "Floor above the Basement"
                                  annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.InternalWall InternalWall4(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=43.416,
        inc=1.5707963267949) "Internal Wall between the Apartemnts of A and B"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-57,40})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{70,-28},{84,-14}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{64,-54},{80,-38}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{64,-78},{80,-62}})));
   equation
     connect(NorthWall.propsBus_a,A01. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,A01. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,A01. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,A01. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,A01. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,A01. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,A01. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,A01. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,A01. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,A01. propsBus[10]) annotation (Line(
         points={{183,-21},{178.5,-21},{178.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,A01. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(FloorAboveBasement.propsBus_b, A01.propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(FloorAboveBasement.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{47.5,71},{47.5,63},{-2,63}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(A01.propsBus[14], zoneBus[3]) annotation (Line(
         points={{0,-12.16},{0,73},{-2,73}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(InternalWall4.propsBus_b, A01.propsBus[12]) annotation (Line(
         points={{-61,35},{-20.5,35},{-20.5,-11.3067},{-1.77636e-015,-11.3067}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
      connect(InternalWall4.propsBus_a, zoneBus[1]) annotation (Line(
          points={{-61,45},{-60.5,45},{-60.5,53},{-2,53}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
     connect(A01.propsBus[15], zoneBus[4]) annotation (Line(
         points={{0,-12.5867},{2,-12.5867},{2,83},{-2,83}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(A01.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36,0},{36,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A01.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(A01.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{56,-16},{56,-21},{77,-21}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(A01.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{52,-20.8},{52,-46},{72,-46}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(A01.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{52,-25.6},{52,-70},{72,-70}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="A01")}));
   end Apartment_A01;

   model Apartment_B12

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        AWall=18.57025) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
        A=7.05275,
        inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
        inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
        frac=0.1385,
        inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall WestWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.EastWest_InsulatedWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
        AWall=21.6,
        inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=65.016,
        inc=1.5707963267949) "Internal Wall between the Apartemnts"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-91,40})));
     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=38.61,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=46.737,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone B12(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall Floor(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Floor"
                                          annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.InternalWall CeilingRoof(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.CeilingRoof constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Ceiling,
       AWall=57.1298,
        inc=IDEAS.Constants.Ceiling) "Ceiling under the Roof"
                                                             annotation (Placement(
           transformation(
           extent={{-5,10},{5,-10}},
           rotation=270,
           origin={89,44})));
     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{72,-30},{88,-14}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{68,-58},{82,-44}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{70,-78},{82,-66}})));
   equation
     connect(NorthWall.propsBus_a,B12. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,B12. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,B12. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,B12. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall.propsBus_a,B12. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,B12. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,B12. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,B12. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,B12. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,B12. propsBus[10]) annotation (Line(
         points={{183,-21},{184.5,-21},{184.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,B12. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_b,B12. propsBus[12]) annotation (Line(
         points={{-95,35},{-16.5,35},{-16.5,-11.3067},{-1.77636e-015,-11.3067}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_a, zoneBus[1]) annotation (Line(
         points={{-95,45},{-94.5,45},{-94.5,53},{-2,53}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(Floor.propsBus_b,B12. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(CeilingRoof.propsBus_a,B12. propsBus[14]) annotation (Line(
         points={{85,39},{-7.5,39},{-7.5,-12.16},{-1.77636e-015,-12.16}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{47.5,71},{47.5,63},{-2,63}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(CeilingRoof.propsBus_b, zoneBus[3]) annotation (Line(
         points={{85,49},{77.5,49},{77.5,73},{-2,73}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(B12.propsBus[15], zoneBus[4]) annotation (Line(
          points={{0,-12.5867},{0,83},{-2,83}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     connect(B12.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36,0},{36,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(B12.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(B12.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{52,-16},{52,-22},{80,-22}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(B12.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{52,-20.8},{52,-51},{75,-51}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(B12.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{58,-25.6},{58,-72},{76,-72}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="B12")}));
   end Apartment_B12;

   model Apartment_B02

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        AWall=17.813) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
        A=7.81,
        inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
        inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
        frac=0.1385,
        inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall WestWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.EastWest_InsulatedWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
        AWall=21.6,
        inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=65.016,
        inc=1.5707963267949) "Internal Wall between the Apartemnts"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-91,40})));
     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=38.61,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=36.342,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone B02(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall FloorAboveBasement(
        insulationThickness=0,
        redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor,
        redeclare BM_ABC.Data.Constructions.FloorAboveBasement constructionType)
        "Floor above the Basement" annotation (Placement(transformation(
            extent={{-5,-10},{5,10}},
            rotation=90,
            origin={53,66})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{74,-30},{90,-14}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{70,-52},{82,-40}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{70,-74},{82,-62}})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
   equation
     connect(NorthWall.propsBus_a,B02. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,B02. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,B02. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,B02. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall.propsBus_a,B02. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,B02. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,B02. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,B02. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,B02. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,B02. propsBus[10]) annotation (Line(
         points={{183,-21},{178.5,-21},{178.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,B02. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_b,B02. propsBus[12]) annotation (Line(
         points={{-95,35},{-16.5,35},{-16.5,-11.3067},{-1.77636e-015,-11.3067}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_a, zoneBus[1]) annotation (Line(
         points={{-95,45},{-94.5,45},{-94.5,53},{-2,53}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(FloorAboveBasement.propsBus_b, B02.propsBus[13]) annotation (Line(
          points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(FloorAboveBasement.propsBus_a, zoneBus[2]) annotation (Line(
          points={{49,71},{47.5,71},{47.5,63},{-2,63}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(B02.propsBus[14], zoneBus[3]) annotation (Line(
          points={{0,-12.16},{0,73},{-2,73}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(B02.propsBus[15], zoneBus[4]) annotation (Line(
          points={{0,-12.5867},{2,-12.5867},{2,83},{-2,83}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     connect(B02.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{60,-16},{60,-22},{82,-22}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(B02.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{54,-20.8},{54,-46},{76,-46}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(B02.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{54,-25.6},{54,-68},{76,-68}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(B02.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36,0},{36,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(B02.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="B02")}));
   end Apartment_B02;

   model Apartment_B11

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=18.57025) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=7.05275,
       inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
       frac=0.1385,
       inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        redeclare BM_ABC.Data.Constructions.EastWest_NonInsulatedWall
          constructionType,
        AWall=43.2,
        azi=IDEAS.Constants.East,
        inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=37.935,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=35.829,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone B11(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall Floor(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Floor" annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.InternalWall CeilingRoof(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.CeilingRoof constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Ceiling,
       AWall=57.1298,
       inc=IDEAS.Constants.Ceiling) "Ceiling under the Roof" annotation (Placement(
           transformation(
           extent={{-5,10},{5,-10}},
           rotation=270,
           origin={89,44})));
     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.InternalWall InternalWall4(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=21.816,
       inc=1.5707963267949) "Internal Wall between the Apartments of B and C"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-51,42})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{74,-30},{90,-14}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{70,-56},{84,-42}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{70,-76},{82,-64}})));
   equation
     connect(NorthWall.propsBus_a,B11. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,B11. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,B11. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,B11. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,B11. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,B11. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,B11. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,B11. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,B11. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,B11. propsBus[10]) annotation (Line(
         points={{183,-21},{184.5,-21},{184.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,B11. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_b,B11. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(CeilingRoof.propsBus_a,B11. propsBus[14]) annotation (Line(
         points={{85,39},{-7.5,39},{-7.5,-12.16},{-1.77636e-015,-12.16}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{49.5,71},{49.5,63},{-2,63}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(CeilingRoof.propsBus_b, zoneBus[3]) annotation (Line(
         points={{85,49},{77.5,49},{77.5,73},{-2,73}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(InternalWall4.propsBus_a, zoneBus[1]) annotation (Line(
          points={{-55,47},{-54.5,47},{-54.5,53},{-2,53}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(InternalWall4.propsBus_b, B11.propsBus[12]) annotation (Line(
          points={{-55,37},{-20.5,37},{-20.5,-11.3067},{-1.77636e-015,-11.3067}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

     connect(B11.propsBus[15], zoneBus[4]) annotation (Line(
         points={{0,-12.5867},{0,-12.5867},{0,83},{-2,83}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(B11.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36,0},{36,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(B11.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(B11.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{60,-16},{60,-22},{82,-22}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(B11.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{58,-20.8},{58,-49},{77,-49}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(B11.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{56,-25.6},{56,-70},{76,-70}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="B11")}));
   end Apartment_B11;

   model Apartment_B01

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=17.813) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=7.81,
       inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
       frac=0.1385,
       inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        redeclare BM_ABC.Data.Constructions.EastWest_NonInsulatedWall
          constructionType,
        azi=IDEAS.Constants.East,
        AWall=43.2,
        inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=38.61,
       inc=1.5707963267949) "Internal Wall Small Brick" annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
       AWall=36.342,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone B01(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall FloorAboveBasement(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.FloorAboveBasement constructionType)
        "Floor above the Basement"
                                  annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.InternalWall InternalWall4(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=21.816,
        inc=1.5707963267949) "Internal Wall between the Apartemnts of B and C"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-55,40})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{76,-30},{90,-16}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{70,-54},{82,-42}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{70,-78},{82,-66}})));
   equation
     connect(NorthWall.propsBus_a,B01. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,B01. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,B01. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,B01. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,B01. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,B01. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,B01. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,B01. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,B01. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,B01. propsBus[10]) annotation (Line(
         points={{183,-21},{178.5,-21},{178.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,B01. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(FloorAboveBasement.propsBus_b,B01. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(FloorAboveBasement.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{47.5,71},{47.5,63},{-2,63}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(B01.propsBus[14], zoneBus[3]) annotation (Line(
         points={{0,-12.16},{0,73},{-2,73}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(InternalWall4.propsBus_b,B01. propsBus[12]) annotation (Line(
         points={{-59,35},{-20.5,35},{-20.5,-11.3067},{-1.77636e-015,-11.3067}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
      connect(InternalWall4.propsBus_a, zoneBus[1]) annotation (Line(
          points={{-59,45},{-58.5,45},{-58.5,53},{-2,53}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
     connect(B01.propsBus[15], zoneBus[4]) annotation (Line(
         points={{0,-12.5867},{2,-12.5867},{2,83},{-2,83}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(B01.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36.8,0},{36.8,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(B01.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(B01.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{60,-16},{60,-23},{83,-23}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(B01.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{59,-20.8},{59,-48},{76,-48}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(B01.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{54,-25.6},{54,-72},{76,-72}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="B01")}));
   end Apartment_B01;

   model Apartment_C12

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        AWall=18.57025) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
        A=7.05275,
        inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
        inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
        frac=0.1385,
        inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall WestWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.EastWest_InsulatedWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
        AWall=43.2,
        inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=65.016,
        inc=1.5707963267949) "Internal Wall between the Apartemnts"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-91,40})));
     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=38.61,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=46.737,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone C12(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall Floor(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Floor"
                                          annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.InternalWall CeilingRoof(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.CeilingRoof constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Ceiling,
       AWall=57.1298,
        inc=IDEAS.Constants.Ceiling) "Ceiling under the Roof"
                                                             annotation (Placement(
           transformation(
           extent={{-5,10},{5,-10}},
           rotation=270,
           origin={89,44})));
     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{88,10},{108,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{76,-30},{92,-14}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{70,-54},{82,-42}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{72,-76},{82,-66}})));
   equation
     connect(NorthWall.propsBus_a,C12. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,C12. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,C12. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,C12. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall.propsBus_a,C12. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,C12. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,C12. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,C12. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,C12. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,C12. propsBus[10]) annotation (Line(
         points={{183,-21},{184.5,-21},{184.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,C12. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_b,C12. propsBus[12]) annotation (Line(
         points={{-95,35},{-16.5,35},{-16.5,-11.3067},{-1.77636e-015,-11.3067}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_a, zoneBus[1]) annotation (Line(
         points={{-95,45},{-94.5,45},{-94.5,53},{-2,53}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(Floor.propsBus_b,C12. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(CeilingRoof.propsBus_a,C12. propsBus[14]) annotation (Line(
         points={{85,39},{-7.5,39},{-7.5,-12.16},{-1.77636e-015,-12.16}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{47.5,71},{47.5,63},{-2,63}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(CeilingRoof.propsBus_b, zoneBus[3]) annotation (Line(
         points={{85,49},{77.5,49},{77.5,73},{-2,73}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(C12.propsBus[15], zoneBus[4]) annotation (Line(
          points={{0,-12.5867},{0,83},{-2,83}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     connect(C12.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36,0},{36,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C12.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{98,0},{98,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C12.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{60,-16},{60,-22},{84,-22}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(C12.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{57,-20.8},{57,-48},{76,-48}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(C12.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{56,-25.6},{56,-71},{77,-71}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="C12")}));
   end Apartment_C12;

   model Apartment_C02

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        AWall=17.813) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
        A=7.81,
        inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
        inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
        frac=0.1385,
        inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall WestWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.EastWest_InsulatedWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
        AWall=43.2,
        inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall1 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=65.016,
        inc=1.5707963267949) "Internal Wall between the Apartemnts"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={-91,40})));
     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=38.61,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=36.342,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone C02(          V=308.5, nSurf=15)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall FloorAboveBasement(
        insulationThickness=0,
        redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        azi=IDEAS.Constants.Floor,
        AWall=57.1298,
        inc=IDEAS.Constants.Floor,
        redeclare BM_ABC.Data.Constructions.FloorAboveBasement constructionType)
        "Floor above the Basement" annotation (Placement(transformation(
            extent={{-5,-10},{5,10}},
            rotation=90,
            origin={53,66})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[4]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{88,10},{108,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{76,-30},{90,-16}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{66,-56},{80,-42}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{68,-78},{78,-68}})));
   equation
     connect(NorthWall.propsBus_a,C02. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.61333},{-1.77636e-015,-6.61333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,C02. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.04},{-1.77636e-015,-7.04}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,C02. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.46667},{-1.77636e-015,-7.46667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,C02. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-7.89333},{-1.77636e-015,-7.89333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall.propsBus_a,C02. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.32},{-1.77636e-015,-8.32}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,C02. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.74667},{-1.77636e-015,-8.74667}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,C02. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.17333},{-1.77636e-015,-9.17333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,C02. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.6},{-1.77636e-015,-9.6}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,C02. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.0267},{-1.77636e-015,-10.0267}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,C02. propsBus[10]) annotation (Line(
         points={{183,-21},{178.5,-21},{178.5,-10.4533},{-1.77636e-015,-10.4533}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,C02. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-10.88},{-1.77636e-015,-10.88}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_b,C02. propsBus[12]) annotation (Line(
         points={{-95,35},{-16.5,35},{-16.5,-11.3067},{-1.77636e-015,-11.3067}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall1.propsBus_a, zoneBus[1]) annotation (Line(
         points={{-95,45},{-94.5,45},{-94.5,53},{-2,53}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(FloorAboveBasement.propsBus_b, C02.propsBus[13]) annotation (Line(
          points={{49,61},{-11.5,61},{-11.5,-11.7333},{-1.77636e-015,-11.7333}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(FloorAboveBasement.propsBus_a, zoneBus[2]) annotation (Line(
          points={{49,71},{47.5,71},{47.5,63},{-2,63}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(C02.propsBus[14], zoneBus[3]) annotation (Line(
          points={{0,-12.16},{0,73},{-2,73}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(C02.propsBus[15], zoneBus[4]) annotation (Line(
          points={{0,-12.5867},{2,-12.5867},{2,83},{-2,83}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     connect(C02.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36,0},{36,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C02.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{98,0},{98,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C02.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{60,-16},{60,-23},{83,-23}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(C02.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{56,-20.8},{56,-49},{73,-49}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(C02.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{54,-25.6},{54,-73},{73,-73}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="C02")}));
   end Apartment_C02;

   model Apartment_C11

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=18.57025) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=7.05275,
       inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
       frac=0.1385,
       inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        redeclare BM_ABC.Data.Constructions.EastWest_NonInsulatedWall
          constructionType,
        azi=IDEAS.Constants.East,
        AWall=65.016,
        inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=37.935,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=35.829,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone C11(          V=308.5, nSurf=14)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall Floor(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Floor" annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.InternalWall CeilingRoof(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.CeilingRoof constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Ceiling,
       AWall=57.1298,
       inc=IDEAS.Constants.Ceiling) "Ceiling under the Roof" annotation (Placement(
           transformation(
           extent={{-5,10},{5,-10}},
           rotation=270,
           origin={89,44})));
     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[3]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{78,-30},{92,-16}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{74,-52},{84,-42}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{74,-74},{84,-64}})));
   equation
     connect(NorthWall.propsBus_a,C11. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.62857},{-1.77636e-015,-6.62857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,C11. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.08571},{-1.77636e-015,-7.08571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,C11. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.54286},{-1.77636e-015,-7.54286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,C11. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-8},{-1.77636e-015,-8}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,C11. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.45714},{-1.77636e-015,-8.45714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,C11. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.91429},{-1.77636e-015,-8.91429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,C11. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.37143},{-1.77636e-015,-9.37143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,C11. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.82857},{-1.77636e-015,-9.82857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,C11. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.2857},{-1.77636e-015,-10.2857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,C11. propsBus[10]) annotation (Line(
         points={{183,-21},{184.5,-21},{184.5,-10.7429},{-1.77636e-015,-10.7429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,C11. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-11.2},{-1.77636e-015,-11.2}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_b,C11. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-12.1143},{-1.77636e-015,-12.1143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(CeilingRoof.propsBus_a,C11. propsBus[14]) annotation (Line(
         points={{85,39},{-7.5,39},{-7.5,-12.5714},{-1.77636e-015,-12.5714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(Floor.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{49.5,71},{49.5,68},{-2,68}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(CeilingRoof.propsBus_b, zoneBus[3]) annotation (Line(
         points={{85,49},{77.5,49},{77.5,81.3333},{-2,81.3333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(C11.propsBus[12], zoneBus[1]) annotation (Line(
          points={{0,-11.6571},{0,-11.6571},{0,54.6667},{-2,54.6667}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
     connect(C11.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36.8,0},{36.8,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C11.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C11.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{62,-16},{62,-23},{85,-23}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(C11.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{59,-20.8},{59,-47},{79,-47}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(C11.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{56,-25.6},{56,-69},{79,-69}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="C11")}));
   end Apartment_C11;

   model Apartment_C01

     IDEAS.Buildings.Components.OuterWall NorthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.NorthWall constructionType,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=17.813) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.Window NorthWindow(
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=7.81,
       inc=1.5707963267949) "North Window"
       annotation (Placement(transformation(extent={{-66,-80},{-56,-60}})));
     IDEAS.Buildings.Components.OuterWall SouthWall(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.SouthWall constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       AWall=8.8562,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-92,-72},{-82,-52}})));
     IDEAS.Buildings.Components.Window SouthWindow(
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Glazing.SouthWindow glazing,
       redeclare IDEAS.Buildings.Data.Interfaces.Frame fraType,
       redeclare IDEAS.Buildings.Components.Shading.None shaType,
       A=16.7668,
       frac=0.1385,
       inc=1.5707963267949) "South Window"
       annotation (Placement(transformation(extent={{-118,-66},{-108,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
        redeclare BM_ABC.Data.Constructions.EastWest_NonInsulatedWall
          constructionType,
        azi=IDEAS.Constants.East,
        AWall=65.016,
        inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-144,-56},{-134,-36}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       AWall=38.61,
       inc=1.5707963267949) "Internal Wall Small Brick" annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={179,-26})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
       AWall=36.342,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={149,-24})));
     IDEAS.Buildings.Components.Zone C01(          V=308.5, nSurf=14)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));
     IDEAS.Buildings.Components.InternalWall InternalCeilingFloor(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Normal_CeilingFloor constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor) "Internal Ceiling/Floor in the Apartment"
       annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={115,-24})));
     IDEAS.Buildings.Components.InternalWall FloorAboveBasement(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.Floor,
       AWall=57.1298,
       inc=IDEAS.Constants.Floor,
       redeclare BM_ABC.Data.Constructions.FloorAboveBasement constructionType)
        "Floor above the Basement"
                                  annotation (Placement(transformation(
           extent={{-5,-10},{5,10}},
           rotation=90,
           origin={53,66})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[3]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Fluid.Interfaces.FlowPort_b flowPort_Out1(redeclare package Medium
          =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{50,10},{70,30}})));
     IDEAS.Fluid.Interfaces.FlowPort_a flowPort_In1(redeclare package Medium =
           IDEAS.Experimental.Media.AirPTDecoupled)
       annotation (Placement(transformation(extent={{90,10},{110,30}})));
     Modelica.Blocks.Interfaces.RealOutput TSensor1
        "Sensor temperature of the zone, i.e. operative temeprature"
       annotation (Placement(transformation(extent={{76,-30},{90,-16}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a gainCon1
        "Internal zone node for convective heat gains"
       annotation (Placement(transformation(extent={{70,-54},{84,-40}})));
     Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b gainRad1
        "Internal zone node for radiative heat gains"
       annotation (Placement(transformation(extent={{72,-78},{84,-66}})));
   equation
     connect(NorthWall.propsBus_a,C01. propsBus[1]) annotation (Line(
         points={{-30,-76},{-24,-76},{-24,-6.62857},{-1.77636e-015,-6.62857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWindow.propsBus_a,C01. propsBus[2]) annotation (Line(
         points={{-56,-66},{-50,-66},{-50,-7.08571},{-1.77636e-015,-7.08571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,C01. propsBus[3]) annotation (Line(
         points={{-82,-58},{-76,-58},{-76,-7.54286},{-1.77636e-015,-7.54286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWindow.propsBus_a,C01. propsBus[4]) annotation (Line(
         points={{-108,-52},{-100,-52},{-100,-8},{-1.77636e-015,-8}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,C01. propsBus[5]) annotation (Line(
         points={{-134,-42},{-116,-42},{-116,-8.45714},{-1.77636e-015,-8.45714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_b,C01. propsBus[6]) annotation (Line(
         points={{119,-19},{118.5,-19},{118.5,-8.91429},{-1.77636e-015,-8.91429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalCeilingFloor.propsBus_a,C01. propsBus[7]) annotation (Line(
         points={{119,-29},{101.5,-29},{101.5,-9.37143},{-1.77636e-015,-9.37143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_b,C01. propsBus[8]) annotation (Line(
         points={{153,-19},{152.5,-19},{152.5,-9.82857},{-1.77636e-015,-9.82857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall3.propsBus_a,C01. propsBus[9]) annotation (Line(
         points={{153,-29},{131.5,-29},{131.5,-10.2857},{-1.77636e-015,-10.2857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_b,C01. propsBus[10]) annotation (Line(
         points={{183,-21},{178.5,-21},{178.5,-10.7429},{-1.77636e-015,-10.7429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(InternalWall2.propsBus_a,C01. propsBus[11]) annotation (Line(
         points={{183,-31},{163.5,-31},{163.5,-11.2},{-1.77636e-015,-11.2}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(FloorAboveBasement.propsBus_b,C01. propsBus[13]) annotation (Line(
         points={{49,61},{-11.5,61},{-11.5,-12.1143},{-1.77636e-015,-12.1143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(FloorAboveBasement.propsBus_a, zoneBus[2]) annotation (Line(
         points={{49,71},{47.5,71},{47.5,68},{-2,68}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
     connect(C01.propsBus[14], zoneBus[3]) annotation (Line(
         points={{0,-12.5714},{0,81.3333},{-2,81.3333}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None), Text(
         string="%second",
         index=1,
         extent={{6,3},{6,3}}));
      connect(C01.propsBus[12], zoneBus[1]) annotation (Line(
          points={{0,-11.6571},{-2,-11.6571},{-2,54.6667}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
     connect(C01.flowPort_Out, flowPort_Out1) annotation (Line(
         points={{13.6,0},{36,0},{36,20},{60,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C01.flowPort_In, flowPort_In1) annotation (Line(
         points={{20.4,0},{100,0},{100,20}},
         color={0,0,0},
         smooth=Smooth.None));
     connect(C01.TSensor, TSensor1) annotation (Line(
         points={{35.02,-16},{60,-16},{60,-23},{83,-23}},
         color={0,0,127},
         smooth=Smooth.None));
     connect(C01.gainCon, gainCon1) annotation (Line(
         points={{34,-20.8},{60,-20.8},{60,-47},{77,-47}},
         color={191,0,0},
         smooth=Smooth.None));
     connect(C01.gainRad, gainRad1) annotation (Line(
         points={{34,-25.6},{56,-25.6},{56,-72},{78,-72}},
         color={191,0,0},
         smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
               -100},{200,100}}), graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-36,14},{52,-40}},
             lineColor={0,0,0},
              textString="C01")}));
   end Apartment_C01;

   model Basement_A

     IDEAS.Buildings.Components.SlabOnGround
                                          NorthWall(
       insulationThickness=0,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_2 constructionType,
       AWall=25.623,
       PWall=24.38) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          SouthWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.BasementWall_2 constructionType,
       AWall=25.623,
       PWall=24.38,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-72,-72},{-62,-52}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=32.508,
       PWall=29.48,
       azi=IDEAS.Constants.East,
       inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-132,-48},{-122,-28}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.BasementWall_3 constructionType,
       AWall=23.058,
       inc=1.5707963267949) "Internal Wall 19cm"        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={97,-32})));
     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.BasementWall_4 constructionType,
       AWall=16.497,
       inc=1.5707963267949) "Internal Wall 29cm"           annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={63,-30})));
     IDEAS.Buildings.Components.Zone Basement_A(V=308.5, nSurf=14)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[2]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          WestWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=32.508,
       PWall=29.48,
       inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-102,-62},{-92,-42}})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=7.182,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={167,-30})));
     IDEAS.Buildings.Components.InternalWall InternalWall4(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=25.758,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={131,-30})));
   equation
     connect(InternalWall1.propsBus_b, Basement_A.propsBus[5]) annotation (Line(
         points={{67,-25},{66.5,-25},{66.5,-8.45714},{-1.77636e-015,-8.45714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall1.propsBus_a, Basement_A.propsBus[6]) annotation (Line(
         points={{67,-35},{77.5,-35},{77.5,-8.91429},{-1.77636e-015,-8.91429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall2.propsBus_b, Basement_A.propsBus[7]) annotation (Line(
         points={{101,-27},{100.5,-27},{100.5,-9.37143},{-1.77636e-015,-9.37143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall2.propsBus_a, Basement_A.propsBus[8]) annotation (Line(
         points={{101,-37},{111.5,-37},{111.5,-9.82857},{-1.77636e-015,-9.82857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWall.propsBus_a, Basement_A.propsBus[1]) annotation (Line(
         points={{-30,-76},{-16,-76},{-16,-6.62857},{0,-6.62857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a, Basement_A.propsBus[2]) annotation (Line(
         points={{-62,-58},{-54,-58},{-54,-7.08571},{-1.77636e-015,-7.08571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall1.propsBus_a, Basement_A.propsBus[3]) annotation (Line(
         points={{-92,-48},{-86,-48},{-86,-7.54286},{-1.77636e-015,-7.54286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a, Basement_A.propsBus[4]) annotation (Line(
         points={{-122,-34},{-114,-34},{-114,-8},{-1.77636e-015,-8}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
      connect(InternalWall4.propsBus_b, Basement_A.propsBus[9]) annotation (
          Line(
          points={{135,-25},{134,-25},{134,-10.2857},{-1.77636e-015,-10.2857}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(InternalWall4.propsBus_a, Basement_A.propsBus[10]) annotation (
          Line(
          points={{135,-35},{148.5,-35},{148.5,-10.7429},{-1.77636e-015,-10.7429}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(InternalWall3.propsBus_b, Basement_A.propsBus[11]) annotation (
          Line(
          points={{171,-25},{170.5,-25},{170.5,-11.2},{-1.77636e-015,-11.2}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(InternalWall3.propsBus_a, Basement_A.propsBus[12]) annotation (
          Line(
          points={{171,-35},{184.5,-35},{184.5,-11.6571},{-1.77636e-015,-11.6571}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Basement_A.propsBus[13], zoneBus[1]) annotation (Line(
          points={{0,-12.1143},{-2,-12.1143},{-2,58}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Basement_A.propsBus[14], zoneBus[2]) annotation (Line(
          points={{0,-12.5714},{2,-12.5714},{2,78},{-2,78}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-38,34},{102,-52}},
             lineColor={0,0,0},
              textString="Basement A")}));
   end Basement_A;

   model Basement_B

     IDEAS.Buildings.Components.SlabOnGround
                                          NorthWall(
       insulationThickness=0,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_2 constructionType,
       AWall=25.623,
       PWall=24.38) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          SouthWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.BasementWall_2 constructionType,
       AWall=25.623,
       PWall=24.38,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-72,-72},{-62,-52}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=32.508,
       PWall=29.48,
       azi=IDEAS.Constants.East,
       inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-132,-48},{-122,-28}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.BasementWall_3 constructionType,
       AWall=23.058,
       inc=1.5707963267949) "Internal Wall 19cm"        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={97,-32})));
     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.BasementWall_4 constructionType,
       AWall=16.497,
       inc=1.5707963267949) "Internal Wall 29cm"           annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={63,-30})));
     IDEAS.Buildings.Components.Zone Basement_B(V=308.5, nSurf=14)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[2]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          WestWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=32.508,
       PWall=29.48,
       inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-102,-62},{-92,-42}})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=7.182,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={167,-30})));
     IDEAS.Buildings.Components.InternalWall InternalWall4(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=25.758,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={131,-30})));
   equation
     connect(InternalWall1.propsBus_b,Basement_B. propsBus[5]) annotation (Line(
         points={{67,-25},{66.5,-25},{66.5,-8.45714},{-1.77636e-015,-8.45714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall1.propsBus_a,Basement_B. propsBus[6]) annotation (Line(
         points={{67,-35},{77.5,-35},{77.5,-8.91429},{-1.77636e-015,-8.91429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall2.propsBus_b,Basement_B. propsBus[7]) annotation (Line(
         points={{101,-27},{100.5,-27},{100.5,-9.37143},{-1.77636e-015,-9.37143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall2.propsBus_a,Basement_B. propsBus[8]) annotation (Line(
         points={{101,-37},{111.5,-37},{111.5,-9.82857},{-1.77636e-015,-9.82857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWall.propsBus_a,Basement_B. propsBus[1]) annotation (Line(
         points={{-30,-76},{-16,-76},{-16,-6.62857},{0,-6.62857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,Basement_B. propsBus[2]) annotation (Line(
         points={{-62,-58},{-54,-58},{-54,-7.08571},{-1.77636e-015,-7.08571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall1.propsBus_a,Basement_B. propsBus[3]) annotation (Line(
         points={{-92,-48},{-86,-48},{-86,-7.54286},{-1.77636e-015,-7.54286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,Basement_B. propsBus[4]) annotation (Line(
         points={{-122,-34},{-114,-34},{-114,-8},{-1.77636e-015,-8}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
      connect(InternalWall4.propsBus_b, Basement_B.propsBus[9]) annotation (
          Line(
          points={{135,-25},{134,-25},{134,-10.2857},{-1.77636e-015,-10.2857}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(InternalWall4.propsBus_a, Basement_B.propsBus[10]) annotation (
          Line(
          points={{135,-35},{148.5,-35},{148.5,-10.7429},{-1.77636e-015,
              -10.7429}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(InternalWall3.propsBus_b, Basement_B.propsBus[11]) annotation (
          Line(
          points={{171,-25},{170.5,-25},{170.5,-11.2},{-1.77636e-015,-11.2}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(InternalWall3.propsBus_a, Basement_B.propsBus[12]) annotation (
          Line(
          points={{171,-35},{184.5,-35},{184.5,-11.6571},{-1.77636e-015,
              -11.6571}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Basement_B.propsBus[13], zoneBus[1]) annotation (Line(
          points={{0,-12.1143},{-2,-12.1143},{-2,58}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Basement_B.propsBus[14], zoneBus[2]) annotation (Line(
          points={{0,-12.5714},{2,-12.5714},{2,78},{-2,78}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-38,34},{96,-48}},
             lineColor={0,0,0},
              textString="Basement B")}));
   end Basement_B;

   model Basement_C

     IDEAS.Buildings.Components.SlabOnGround
                                          NorthWall(
       insulationThickness=0,
       inc=IDEAS.Constants.Wall,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_2 constructionType,
       AWall=25.623,
       PWall=24.38) "NorthWall"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          SouthWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.BasementWall_2 constructionType,
       AWall=25.623,
       PWall=24.38,
       inc=1.5707963267949) "South Wall"
       annotation (Placement(transformation(extent={{-72,-72},{-62,-52}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=32.508,
       PWall=29.48,
       azi=IDEAS.Constants.East,
       inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-132,-48},{-122,-28}})));

     IDEAS.Buildings.Components.InternalWall InternalWall2(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.BasementWall_3 constructionType,
       AWall=23.058,
       inc=1.5707963267949) "Internal Wall 19cm"        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={97,-32})));
     IDEAS.Buildings.Components.InternalWall InternalWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.BasementWall_4 constructionType,
       AWall=16.497,
       inc=1.5707963267949) "Internal Wall 29cm"           annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={63,-30})));
     IDEAS.Buildings.Components.Zone Basement_C(V=308.5, nSurf=14)
       annotation (Placement(transformation(extent={{0,-32},{34,0}})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[2]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.SlabOnGround
                                          WestWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=32.508,
       PWall=29.48,
       inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-102,-62},{-92,-42}})));
     IDEAS.Buildings.Components.InternalWall InternalWall3(
       insulationThickness=0,
       redeclare BM_ABC.Data.Constructions.Internal_Wall3 constructionType,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
        AWall=7.182,
        inc=1.5707963267949) "Internal Wall Small Brick"
                                                        annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={167,-30})));
     IDEAS.Buildings.Components.InternalWall InternalWall4(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.East,
       redeclare BM_ABC.Data.Constructions.Internal_Wall2 constructionType,
        AWall=25.758,
        inc=1.5707963267949) "Internal Wall Big Brick"     annotation (Placement(
           transformation(
           extent={{-5,-10},{5,10}},
           rotation=-90,
           origin={131,-30})));
   equation
     connect(InternalWall1.propsBus_b,Basement_C. propsBus[5]) annotation (Line(
         points={{67,-25},{66.5,-25},{66.5,-8.45714},{-1.77636e-015,-8.45714}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall1.propsBus_a,Basement_C. propsBus[6]) annotation (Line(
         points={{67,-35},{77.5,-35},{77.5,-8.91429},{-1.77636e-015,-8.91429}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall2.propsBus_b,Basement_C. propsBus[7]) annotation (Line(
         points={{101,-27},{100.5,-27},{100.5,-9.37143},{-1.77636e-015,-9.37143}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));

     connect(InternalWall2.propsBus_a,Basement_C. propsBus[8]) annotation (Line(
         points={{101,-37},{111.5,-37},{111.5,-9.82857},{-1.77636e-015,-9.82857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(NorthWall.propsBus_a,Basement_C. propsBus[1]) annotation (Line(
         points={{-30,-76},{-16,-76},{-16,-6.62857},{0,-6.62857}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(SouthWall.propsBus_a,Basement_C. propsBus[2]) annotation (Line(
         points={{-62,-58},{-54,-58},{-54,-7.08571},{-1.77636e-015,-7.08571}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(WestWall1.propsBus_a,Basement_C. propsBus[3]) annotation (Line(
         points={{-92,-48},{-86,-48},{-86,-7.54286},{-1.77636e-015,-7.54286}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
     connect(EastWall.propsBus_a,Basement_C. propsBus[4]) annotation (Line(
         points={{-122,-34},{-114,-34},{-114,-8},{-1.77636e-015,-8}},
         color={255,204,51},
         thickness=0.5,
         smooth=Smooth.None));
      connect(InternalWall4.propsBus_b, Basement_C.propsBus[9]) annotation (
          Line(
          points={{135,-25},{134,-25},{134,-10.2857},{-1.77636e-015,-10.2857}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(InternalWall4.propsBus_a, Basement_C.propsBus[10]) annotation (
          Line(
          points={{135,-35},{148.5,-35},{148.5,-10.7429},{-1.77636e-015,
              -10.7429}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(InternalWall3.propsBus_b, Basement_C.propsBus[11]) annotation (
          Line(
          points={{171,-25},{170.5,-25},{170.5,-11.2},{-1.77636e-015,-11.2}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(InternalWall3.propsBus_a, Basement_C.propsBus[12]) annotation (
          Line(
          points={{171,-35},{184.5,-35},{184.5,-11.6571},{-1.77636e-015,
              -11.6571}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Basement_C.propsBus[13], zoneBus[1]) annotation (Line(
          points={{0,-12.1143},{-2,-12.1143},{-2,58}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Basement_C.propsBus[14], zoneBus[2]) annotation (Line(
          points={{0,-12.5714},{2,-12.5714},{2,78},{-2,78}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-38,34},{104,-50}},
             lineColor={0,0,0},
              textString="Basement C")}));
   end Basement_C;

   model Attic_A

     IDEAS.Buildings.Components.OuterWall NorthRoof(
       insulationThickness=0,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=97.114,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
       inc=0.34906585039887) "North Roof"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.OuterWall SouthRoof(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
        AWall=62.430,
        inc=0.5235987755983) "South Roof"
       annotation (Placement(transformation(extent={{-96,-66},{-86,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       azi=IDEAS.Constants.East,
       AWall=27.646,
        inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-144,-36},{-134,-16}})));

     IDEAS.Buildings.Components.Zone Attic_A(nSurf=7, V=262.365)
        annotation (Placement(transformation(extent={{0,-32},{34,0}})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[2]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.OuterWall WestWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=27.646,
        inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-124,-50},{-114,-30}})));
     IDEAS.Buildings.Components.OuterWall SouthRoof1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
        AWall=11.8625,
        inc=1.5707963267949) "South Roof 1"
       annotation (Placement(transformation(extent={{-76,-76},{-66,-56}})));
   equation

      connect(NorthRoof.propsBus_a, Attic_A.propsBus[1]) annotation (Line(
          points={{-30,-76},{-16,-76},{-16,-6.85714},{0,-6.85714}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Attic_A.propsBus[6], zoneBus[1]) annotation (Line(
          points={{0,-11.4286},{-2,-11.4286},{-2,58}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Attic_A.propsBus[7], zoneBus[2]) annotation (Line(
          points={{0,-12.3429},{2,-12.3429},{2,78},{-2,78}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(WestWall1.propsBus_a, Attic_A.propsBus[4]) annotation (Line(
          points={{-114,-36},{-86,-36},{-86,-9.6},{-1.77636e-015,-9.6}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(EastWall.propsBus_a, Attic_A.propsBus[5]) annotation (Line(
          points={{-134,-22},{-114,-22},{-114,-10.5143},{-1.77636e-015,-10.5143}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(SouthRoof1.propsBus_a, Attic_A.propsBus[2]) annotation (Line(
          points={{-66,-62},{-34,-62},{-34,-7.77143},{0,-7.77143}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(SouthRoof.propsBus_a, Attic_A.propsBus[3]) annotation (Line(
          points={{-86,-52},{-76,-52},{-76,-8.68571},{-1.77636e-015,-8.68571}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-38,34},{102,-52}},
             lineColor={0,0,0},
             textString="Attic A")}));
   end Attic_A;

   model Attic_B

     IDEAS.Buildings.Components.OuterWall NorthRoof(
       insulationThickness=0,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=97.114,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
       inc=0.34906585039887) "North Roof"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.OuterWall SouthRoof(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
        AWall=62.430,
        inc=0.5235987755983) "South Roof"
       annotation (Placement(transformation(extent={{-94,-66},{-84,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       azi=IDEAS.Constants.East,
       AWall=27.646,
       inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-138,-36},{-128,-16}})));

     IDEAS.Buildings.Components.Zone Attic_B(nSurf=7, V=262.365)
        annotation (Placement(transformation(extent={{0,-32},{34,0}})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[2]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.OuterWall WestWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=27.646,
       inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-118,-52},{-108,-32}})));
     IDEAS.Buildings.Components.OuterWall SouthRoof1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
        AWall=11.8625,
        inc=1.5707963267949) "South Roof"
       annotation (Placement(transformation(extent={{-68,-80},{-58,-60}})));
   equation

      connect(NorthRoof.propsBus_a, Attic_B.propsBus[1]) annotation (Line(
          points={{-30,-76},{-16,-76},{-16,-6.85714},{0,-6.85714}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Attic_B.propsBus[6], zoneBus[1]) annotation (Line(
          points={{0,-11.4286},{-2,-11.4286},{-2,58}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Attic_B.propsBus[7], zoneBus[2]) annotation (Line(
          points={{0,-12.3429},{2,-12.3429},{2,78},{-2,78}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(WestWall1.propsBus_a, Attic_B.propsBus[4]) annotation (Line(
          points={{-108,-38},{-86,-38},{-86,-9.6},{-1.77636e-015,-9.6}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(EastWall.propsBus_a, Attic_B.propsBus[5]) annotation (Line(
          points={{-128,-22},{-114,-22},{-114,-10.5143},{-1.77636e-015,-10.5143}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(SouthRoof1.propsBus_a, Attic_B.propsBus[2]) annotation (Line(
          points={{-58,-66},{-50,-66},{-50,-7.77143},{-1.77636e-015,-7.77143}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(SouthRoof.propsBus_a, Attic_B.propsBus[3]) annotation (Line(
          points={{-84,-52},{-72,-52},{-72,-8.68571},{-1.77636e-015,-8.68571}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-38,34},{102,-52}},
             lineColor={0,0,0},
              textString="Attic B")}));
   end Attic_B;

   model Attic_C

     IDEAS.Buildings.Components.OuterWall NorthRoof(
       insulationThickness=0,
       azi=IDEAS.Constants.North,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       AWall=97.114,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
       inc=0.34906585039887) "North Roof"
       annotation (Placement(transformation(extent={{-40,-90},{-30,-70}})));
     IDEAS.Buildings.Components.OuterWall SouthRoof(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
        AWall=62.430,
        inc=0.5235987755983) "South Roof"
       annotation (Placement(transformation(extent={{-92,-66},{-82,-46}})));
     IDEAS.Buildings.Components.OuterWall EastWall(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       azi=IDEAS.Constants.East,
       AWall=27.646,
       inc=1.5707963267949) "East Wall"
       annotation (Placement(transformation(extent={{-136,-36},{-126,-16}})));

     IDEAS.Buildings.Components.Zone Attic_C(nSurf=7, V=262.365)
        annotation (Placement(transformation(extent={{0,-32},{34,0}})));

     IDEAS.Buildings.Components.Interfaces.ZoneBus zoneBus[2]
       annotation (Placement(transformation(extent={{-22,48},{18,88}}),
           iconTransformation(extent={{-22,48},{18,88}})));
     IDEAS.Buildings.Components.OuterWall WestWall1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.West,
       redeclare BM_ABC.Data.Constructions.BasementWall_1 constructionType,
       AWall=27.646,
       inc=1.5707963267949) "West Wall"
       annotation (Placement(transformation(extent={{-118,-54},{-108,-34}})));
     IDEAS.Buildings.Components.OuterWall SouthRoof1(
       insulationThickness=0,
       redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
       azi=IDEAS.Constants.South,
       redeclare BM_ABC.Data.Constructions.LightRoof constructionType,
        AWall=11.8625,
        inc=1.5707963267949) "South Roof"
       annotation (Placement(transformation(extent={{-66,-78},{-56,-58}})));
   equation

      connect(NorthRoof.propsBus_a, Attic_C.propsBus[1]) annotation (Line(
          points={{-30,-76},{-16,-76},{-16,-6.85714},{0,-6.85714}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Attic_C.propsBus[6], zoneBus[1]) annotation (Line(
          points={{0,-11.4286},{-2,-11.4286},{-2,58}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Attic_C.propsBus[7], zoneBus[2]) annotation (Line(
          points={{0,-12.3429},{2,-12.3429},{2,78},{-2,78}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(WestWall1.propsBus_a, Attic_C.propsBus[4]) annotation (Line(
          points={{-108,-40},{-98,-40},{-98,-9.6},{-1.77636e-015,-9.6}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(EastWall.propsBus_a, Attic_C.propsBus[5]) annotation (Line(
          points={{-126,-22},{-114,-22},{-114,-10.5143},{-1.77636e-015,-10.5143}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(SouthRoof.propsBus_a, Attic_C.propsBus[3]) annotation (Line(
          points={{-82,-52},{-74,-52},{-74,-8.68571},{-1.77636e-015,-8.68571}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));

      connect(SouthRoof1.propsBus_a, Attic_C.propsBus[4]) annotation (Line(
          points={{-56,-64},{-50,-64},{-50,-9.6},{-1.77636e-015,-9.6}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
     annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}),graphics), Icon(coordinateSystem(
             preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
           graphics={Rectangle(extent={{-48,68},{118,-86}}, lineColor={0,0,255}), Text(
             extent={{-38,34},{102,-52}},
             lineColor={0,0,0},
              textString="Attic C")}));
   end Attic_C;

 end Blocks;
  annotation (uses(Modelica(version="3.2.1"), IDEAS(version="0.2")));
end BM_ABC;
