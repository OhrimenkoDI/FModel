program GatePlusIntegrationTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, System.IniFiles, Vcl.Forms, Vcl.ComCtrls, Vcl.StdCtrls,
  uMain in '..\uMain.pas', uParameters in '..\uParameters.pas',
  uGateNavigationPlus in '..\uGateNavigationPlus.pas';
procedure Check(B:Boolean;const S:string);
begin if not B then raise Exception.Create(S);end;
procedure Near(A,B:Double;const S:string);
begin Check(Abs(A-B)<1E-8,S);end;
procedure Configure(Hold:Integer);
var Ini:TMemIniFile;V:TParameterValues;Name:string;
begin
 Name:=ChangeFileExt(ParamStr(0),'.ini'); if FileExists(Name) then DeleteFile(Name);
 Ini:=TMemIniFile.Create(Name);
 try
  V:=DefaultParameterValues;
  if Hold=1 then begin V[63]:=0;V[64]:=0;V[65]:=0;end;
  if Hold=2 then begin V[22]:=0;V[23]:=0;V[24]:=0;end;
  WriteParameterValues(Ini,V);Ini.WriteBool('Simulation','AutoStopEnabled',True);Ini.UpdateFile;
 finally Ini.Free;end;
end;
procedure Run(Hold:Integer);
var F:TForm2;A,B,H:TSimulationFrame;I:Integer;T:Double;
begin
 Configure(Hold);F:=TForm2.Create(nil);
 try
  F.ClientWidth:=600;F.ClientHeight:=400;
  F.StartStopButtonClick(nil);
  for I:=1 to 1600 do begin
   F.Integrate(0.01);A:=F.DisplayFrame;
   Check(A.StoppedOnDistance=(A.PIDResult.Finished and A.GateResult.Finished and
    A.AdaptiveResult.Finished and A.GatePlusResult.Finished),'Four-way stop condition');
   if ((Hold=0) and A.StoppedOnDistance) or
      ((Hold=1) and A.PIDResult.Finished and A.GateResult.Finished and A.AdaptiveResult.Finished) or
      ((Hold=2) and A.GatePlusResult.Finished) then Break;
  end;
  if Hold=0 then begin
   Check(A.StoppedOnDistance,'All four should complete');
   Check(not F.IntegratorTimer.Enabled,'Timer after four results');
   Check(Pos('СВП+',F.ResultLabel.Caption)>0,'Missing SVP+ result');
   T:=F.SimulationTime;F.StartStopButtonClick(nil);F.Integrate(0.01);
   Near(F.SimulationTime,T,'Completed run must not advance');
  end else begin
   Check(F.IntegratorTimer.Enabled,'Timer stopped with an active model');
   F.StartStopButtonClick(nil);F.StartStopButtonClick(nil);
   for I:=1 to 400 do F.Integrate(0.01);
   B:=F.DisplayFrame;
   if Hold=1 then begin
    Check(not B.GatePlusResult.Finished,'Holding SVP+ unexpectedly finished');
    Check(B.GatePlus.X>A.GatePlus.X+10,'Fourth model did not continue');
    Check(B.GatePlusReferencePoint.X>A.GatePlusReferencePoint.X,'Fourth target did not continue');
    Check(CompareMem(@A.Gate,@B.Gate,SizeOf(A.Gate)),'Original gate changed after stop');
    Check(CompareMem(@A.Model,@B.Model,SizeOf(A.Model)),'PID changed after stop');
    Check(CompareMem(@A.AdaptiveModel,@B.AdaptiveModel,SizeOf(A.AdaptiveModel)),'Adaptive changed after stop');
   end else begin
    Check(A.GatePlusResult.Finished,'SVP+ should finish first');
    Check(not B.GateResult.Finished and (B.Gate.X>A.Gate.X+10),'Original gate did not continue');
    Check(CompareMem(@A.GatePlus,@B.GatePlus,SizeOf(A.GatePlus)),'Fourth state changed after stop');
    Check(CompareMem(@A.GatePlusTrajectory,@B.GatePlusTrajectory,SizeOf(A.GatePlusTrajectory)),'Fourth path changed');
    Check(CompareMem(@A.GatePlusResult,@B.GatePlusResult,SizeOf(A.GatePlusResult)),'Fourth result changed');
    Near(A.GatePlusReferencePoint.X,B.GatePlusReferencePoint.X,'Fourth target moved after stop');
   end;
  end;
  H:=F.HistoryFrame(0);F.StartStopButton.Caption:='Стоп';
  if F.IntegratorTimer.Enabled then F.StartStopButtonClick(nil);
  F.HistorySlider.Position:=0;F.HistorySliderChange(nil);B:=F.DisplayFrame;
  Check(CompareMem(@H.GatePlus,@B.GatePlus,SizeOf(H.GatePlus)),'Fourth history state');
  Near(H.GatePlusReferencePoint.X,B.GatePlusReferencePoint.X,'Fourth history target');
  Check(B.GatePlusTrajectoryCount=H.GatePlusTrajectoryCount,'Fourth history path');
  F.InitButtonClick(nil);B:=F.DisplayFrame;
  Check(not B.GatePlusResult.Finished and (B.GatePlusTrajectoryCount=0),'Fourth reset result/path');
  Check(B.GatePlus.Controller.Mode=gpmGuidance,'Fourth mode reset');
  Near(B.GatePlus.X,0,'Fourth reset location');
  Near(B.GatePlus.Parameters.SpeedKmh,F.Model.SpeedKmh,'Shared speed');
  Near(B.GatePlusReferencePoint.X,F.Model.ReferencePoint.X,'Shared initial target');
  Writeln('PASS integration hold=',Hold);
 finally F.Free;end;
end;
procedure Settings;
var Ini:TMemIniFile;V:TParameterValues;D:TParametersForm;Bad,I:Integer;Pages:TPageControl;
begin
 Ini:=TMemIniFile.Create('');
 try
  Ini.WriteString('GateNavigation','N','3');Ini.WriteString('GateNavigation','KLambda','0.2');
  Ini.WriteString('GateNavigation','ObserverSlipTime','0.8');
  V:=DefaultParameterValues;ReadParameterValues(Ini,V);
  Near(V[63],3,'Legacy SVP+ common migration');Near(V[54],0.8,'Legacy SVP+ observer migration');
  Ini.WriteString('GateNavigationPlus','N','5');
  V:=DefaultParameterValues;ReadParameterValues(Ini,V);
  Near(V[22],3,'Original N preserved');Near(V[63],5,'Independent SVP+ N');
  Near(V[65],0.5,'No cross-section fallback after migration');
  WriteParameterValues(Ini,V);V:=DefaultParameterValues;ReadParameterValues(Ini,V);
  Near(V[22],3,'Original N roundtrip');Near(V[63],5,'SVP+ N roundtrip');
 finally Ini.Free;end;
 D:=TParametersForm.Create(nil);
 try
  D.SetValues(DefaultParameterValues);Pages:=nil;
  for I:=0 to D.ComponentCount-1 do if D.Components[I] is TPageControl then Pages:=TPageControl(D.Components[I]);
  Check(Assigned(Pages) and (Pages.PageCount=9),'Separate settings tab');
  Check(Pages.Pages[7].Caption='СВП+','SVP+ tab name');
  TEdit(D.FindComponent('Parameter73')).Text:='1.5';
  Check(not D.TryGetValues(V,Bad) and (Bad=73),'Fractional SVP+ delay');
  Pages.ActivePageIndex:=7;TButton(D.FindComponent('DefaultsButton')).Click;
  Check(D.TryGetValues(V,Bad),'Reset SVP+ defaults');
 finally D.Free;end;
end;
begin
 try
  Application.Initialize;Settings;Run(0);Run(1);Run(2);
  DeleteFile(ChangeFileExt(ParamStr(0),'.ini'));
  Writeln('PASS: four independent trajectories, targets, results, history, pause/reset and settings');
 except on E:Exception do begin Writeln(E.ClassName,': ',E.Message);ExitCode:=1;end;end;
end.