program AdaptiveGlideTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Classes, System.Math, System.IniFiles, Vcl.Forms,
 uMain in '..\uMain.pas', uAdaptiveGlide in '..\uAdaptiveGlide.pas', uParameters in '..\uParameters.pas';
procedure Check(B:Boolean; const S:string);
begin if not B then raise Exception.Create(S); end;
procedure Near(A,B:Double; const S:string);
begin Check(Abs(A-B)<=1E-6*(1+Abs(B)),S+Format(' got %.12g expected %.12g',[A,B])); end;
var C,Other:TAdaptiveState; Lines:TStringList; Parts:TArray<string>; Actual:array[0..8]of Double;
 I,J:Integer; Dt,Angle,Gyro,W:Double; F:TForm2; First,Last:TSimulationFrame;
 Ini:TMemIniFile; Values,Loaded:TParameterValues; Name:string; Saved:Boolean;
begin
 try
  C.Init;
  // Golden vectors use the original C++ coefficients, independently of UI defaults.
  C.Parameters.SlidingGain:=10; C.Parameters.SwitchingGain:=3;
  C.Parameters.Kappa:=-2; C.Parameters.Lambda:=10;
  C.Parameters.IdentifierAlpha:=5; C.Parameters.CovarianceScale:=1000;
  C.Parameters.DenominatorGuard:=0.9; C.Parameters.AngleLimit:=0.7;
  C.Parameters.ReferenceCoefficients[0]:=14.2;
  C.Parameters.ReferenceCoefficients[1]:=51;
  C.Parameters.ReferenceCoefficients[2]:=90;
  C.Reset; Lines:=TStringList.Create;
  try
   Lines.LoadFromFile('tests\adaptive_reference.txt');
   Check(Lines.Count=600,'Reference fixture length');
   for I:=0 to Lines.Count-1 do
   begin
    Dt:=0.003+(I mod 3)*0.001; Gyro:=5+2*Sin(I*0.03); Angle:=0.2+0.1*Sin(I*0.013);
    W:=C.Update(Dt,RadToDeg(Angle),Gyro);
    Parts:=Lines[I].Split([' '],TStringSplitOptions.ExcludeEmpty);
    Actual[0]:=C.Command; Actual[1]:=C.RawReference; Actual[2]:=C.FilteredReference;
    Actual[3]:=C.SlidingSurface; Actual[4]:=C.Residual;
    for J:=0 to 3 do Actual[J+5]:=C.Theta[J];
    for J:=0 to 8 do Near(Actual[J],StrToFloat(Parts[J],TFormatSettings.Invariant),Format('C++ sample %d field %d',[I,J]));
    Near(W,RadToDeg(C.Parameters.RateGain*C.Command),'Rate conversion');
   end;
  finally Lines.Free; end;
  Other:=C; C.Update(0,10,0); Check(CompareMem(@C,@Other,SizeOf(C)),'Zero dt changed state');
  C.Reset; Check(C.Parameters.RateGain=-5,'Reset lost parameters');
  Near(C.Heading,0,'Reset gyro'); Near(C.Theta[3],-10,'Reset theta');
  C.Update(0.005,190,0); Near(C.RawReference,-0.7,'Angle wrapping');
  C.Reset; C.Update(0.005,20,0);
  for I:=1 to 100 do begin W:=C.Update(0.005,20,0); Check(Abs(W)<=RadToDeg(1.5)+1E-9,'Command bound'); end;
  Check(W>0,'Positive angle should turn positive with the configured negative mapping');
  Other:=C;
  try C.Update(-1,0,0); Check(False,'Negative dt accepted'); except on E:EArgumentException do ; end;
  Check(CompareMem(@C,@Other,SizeOf(C)),'Rejected input changed state');
  Ini:=TMemIniFile.Create('');
  try
   Values:=DefaultParameterValues; Values[44]:=-3; Values[52]:=7; Values[53]:=2;
   WriteParameterValues(Ini,Values); Loaded:=DefaultParameterValues; ReadParameterValues(Ini,Loaded);
   for I:=36 to High(Values) do Near(Values[I],Loaded[I],'Adaptive INI');
  finally Ini.Free; end;
  Application.Initialize;
  Name:=ChangeFileExt(ParamStr(0),'.ini'); DeleteFile(Name);
  F:=TForm2.Create(nil);
  try
   Saved:=False; F.StartStopButtonClick(nil);
   for I:=1 to 2000 do
   begin
    F.Integrate(0.01); Last:=F.DisplayFrame;
    Check(Last.AdaptiveTrajectoryCount<=300,'Adaptive path capacity');
    Check(Last.StoppedOnDistance = (Last.PIDResult.Finished and Last.GateResult.Finished and Last.AdaptiveResult.Finished and Last.GatePlusResult.Finished),'Four-way stop condition');
    Check(F.IntegratorTimer.Enabled <> Last.StoppedOnDistance,'Timer stopped before all four results');
    if Last.AdaptiveResult.Finished then
    begin
     if not Saved then begin First:=Last; Saved:=True; end;
     Check(CompareMem(@First.AdaptiveModel,@Last.AdaptiveModel,SizeOf(First.AdaptiveModel)),'Completed adaptive model changed');
     Check(CompareMem(@First.AdaptiveController,@Last.AdaptiveController,SizeOf(First.AdaptiveController)),'Completed adaptive controller changed');
     Check(CompareMem(@First.AdaptiveTrajectory,@Last.AdaptiveTrajectory,SizeOf(First.AdaptiveTrajectory)),'Completed adaptive path changed');
    end;
    if Last.StoppedOnDistance then Break;
   end;
   Writeln('Integration: PID=',Last.PIDResult.Finished,' gate=',Last.GateResult.Finished,
     ' adaptive=',Last.AdaptiveResult.Finished,' adaptive min=',Last.AdaptiveMinimumDistance:0:4);
   Check(Saved,'Adaptive approach did not finish');
   Check(Last.StoppedOnDistance,'All approaches did not finish');
   Check(not F.IntegratorTimer.Enabled,'Timer continued after four results');
   F.HistorySlider.Position:=0; F.HistorySliderChange(nil);
   Near(F.DisplayFrame.AdaptiveModel.X,F.HistoryFrame(0).AdaptiveModel.X,'History adaptive model');
   F.InitButtonClick(nil); Last:=F.DisplayFrame;
   Check(not Last.AdaptiveResult.Finished and (Last.AdaptiveTrajectoryCount=0),'Adaptive reset');
   Near(Last.AdaptiveModel.SpeedKmh,Last.Model.SpeedKmh,'Shared speed');
   Near(Last.AdaptiveModel.InitialHeadingDegrees,Last.Model.InitialHeadingDegrees,'Shared heading');
  finally F.Free; end;
  Writeln('PASS: C++ parity, sensor-only adapter, sign/limits/reset, INI, third trajectory/history/stop');
 except on E:Exception do begin Writeln(E.ClassName,': ',E.Message); ExitCode:=1; end; end;
end.
