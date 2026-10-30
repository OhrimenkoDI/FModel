program GateNavigationTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, System.IniFiles, Vcl.Forms, Vcl.StdCtrls,
  uGateNavigation in '..\uGateNavigation.pas', uMain in '..\uMain.pas',
  uParameters in '..\uParameters.pas';
procedure Check(B: Boolean; const S: string);
begin if not B then raise Exception.Create(S); end;
procedure Near(A, B, E: Double; const S: string);
begin Check(Abs(A-B) <= E, S); end;
var
  P: TGateParameters; C: TGateController; M, Other: TGateSimulation;
  I: Integer; U, Heading, T, Dt, OldCommand, A, Stamp: Double;
  Angles: array[0..1000] of Double;
  Ini: TMemIniFile; Loaded: TParameterValues;
  F: TForm2; Frame: TSimulationFrame; D: TParametersForm;
  Values: TParameterValues; Bad: Integer;
begin
  try
    P := DefaultGateParameters;
    P.SensorDelay := 3;
    C.Reset; Heading := 0; T := 0;
    // Rotating body, fixed inertial LOS: delayed angle + gyro must cancel.
    for I := 0 to 1000 do
    begin
      Dt := 0.005 + (I mod 3)*0.002;
      Angles[I] := 30 - Heading;

      A := 0;
      if I >= 3 then A := Angles[I-3];
      U := C.Update(P,Dt,A,20,I>=3,False);
      Check(Abs(U)<=P.MaxRate,'Output limit');
      Heading := Heading + 20*Dt; T := T+Dt;
    end;
    Near(C.LOSRate,0,1E-8,'Gyro history / varying dt');
    Near(C.CorrectedAngle,30-(Heading-20*Dt),1E-7,'Delay correction');
    // Constant LOS rate across the +/-180 boundary.
    C.Reset; P.SensorDelay := 0;
    for I := 0 to 1000 do
    begin
      A := RadToDeg(ArcTan2(Sin(DegToRad(170+I*0.1)),Cos(DegToRad(170+I*0.1))));
      C.Update(P,0.01,A,0,True,False);
    end;
    Near(C.LOSRate,10,0.001,'Unwrapped LOS rate');
    OldCommand := C.CommandRate;
    C.Update(P,0.01,-80,0,True,True);
    C.Update(P,0.01,-50,0,True,False);
    Near(C.CommandRate,OldCommand,0,'Latched freeze');
    C.Reset;
    Check(not C.Frozen and (C.Output=0) and (C.LOSRate=0),'Controller reset');
    M.Parameters := DefaultGateParameters;
    M.Reset(100,0);
    for I := 1 to 400 do M.Step(0.01,100,0,0,0);
    Near(M.MinimumDistance,0,1E-8,'Straight gate crossing');
    Check(M.Controller.Frozen,'Near zone freeze');
    M.Reset(100,100); Other := M;
    for I := 1 to 700 do M.Step(0.01,100,100,0,0);
    Writeln('Stationary target miss: ',M.MinimumDistance:0:4,' m');
    Check(M.MinimumDistance<5,'Turning approach');
    Near(Other.X,0,0,'Independent states');
    M.Reset(100,100); T := 0;
    for I := 1 to 700 do
    begin
      M.Step(0.01,100+T*10,100,10,0); T := T+0.01;
    end;
    Writeln('Moving target miss: ',M.MinimumDistance:0:4,' m');
    Check(M.MinimumDistance<5,'Moving approach');
    Application.Initialize;
    F := TForm2.Create(nil);
    try
      for I := 1 to 320 do F.Integrate(0.005);
      Check(F.TrajectoryCount=300,'Trajectory capacity');
      Frame := F.HistoryFrame(0);
      F.HistorySlider.Position := 0; F.HistorySliderChange(nil);
      Near(F.DisplayFrame.Gate.X,Frame.Gate.X,0,'Gate history playback');
      Near(F.DisplayFrame.GateTrajectory[0].X,Frame.GateTrajectory[0].X,0,'Path history');
      Stamp := F.Gate.X;
      Check(Abs(Stamp-Frame.Gate.X)>0.01,'History independent of live state');
      F.InitButtonClick(nil);
      Near(F.Gate.X,0,0,'Model reset');
      Check(not F.Gate.Controller.Frozen,'Freeze reset');
    finally F.Free; end;
    Ini := TMemIniFile.Create('');
    try
      Values := DefaultParameterValues;
      Values[22] := 3.5; Values[23] := 0.6; Values[33] := 7; Values[34] := -3;
      WriteParameterValues(Ini, Values);
      Loaded := DefaultParameterValues;
      ReadParameterValues(Ini, Loaded);
      for I := 22 to High(Values) do Near(Values[I], Loaded[I], 1E-10, 'Gate INI round trip');
    finally Ini.Free; end;
    D := TParametersForm.Create(nil);
    try
      Values := DefaultParameterValues; D.SetValues(Values);
      Check(D.TryGetValues(Values,Bad),'Default parameter validation');
      TEdit(D.FindComponent('Parameter33')).Text := '1.5';
      Check(not D.TryGetValues(Values,Bad) and (Bad=33),'Fractional gate delay rejected');
      D.SetValues(DefaultParameterValues);
      TEdit(D.FindComponent('Parameter25')).Text := '0';
      Check(not D.TryGetValues(Values,Bad) and (Bad=25),'Zero max rate rejected');
    finally D.Free; end;
    Writeln('PASS: delay, wrap, saturation, freeze, reset, approaches, independence, history, validation');
  except on E: Exception do begin Writeln(E.ClassName,': ',E.Message); ExitCode:=1; end; end;
end.
