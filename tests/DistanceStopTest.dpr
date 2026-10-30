program DistanceStopTest;
{$APPTYPE CONSOLE}
uses
  System.SysUtils, System.Math, System.IniFiles, Vcl.Forms, Vcl.Imaging.pngimage,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas', uPID in '..\uPID.pas';
procedure Check(B: Boolean; const S: string);
begin if not B then raise Exception.Create(S); end;
procedure Near(A, B: Double; const S: string);
begin Check(Abs(A-B)<1E-9,S); end;
procedure Configure(GateSpeed, TargetX, Limit: Double; Enabled: Boolean);
var Ini: TMemIniFile; Name: string;
begin
  Name:=ChangeFileExt(ParamStr(0),'.ini');
  if FileExists(Name) then DeleteFile(Name);
  Ini:=TMemIniFile.Create(Name);
  try
    Ini.WriteBool('Simulation','AutoStopEnabled',Enabled);
    Ini.WriteString('Simulation','AutoStopDistance',FloatToStr(Limit,TFormatSettings.Invariant));
    Ini.WriteString('ReferencePoint','X',FloatToStr(TargetX,TFormatSettings.Invariant));
    Ini.WriteString('ReferencePoint','Y','0');
    Ini.WriteString('ReferencePoint','SpeedKmh','3.6');
    Ini.WriteString('ReferencePoint','HeadingDegrees','0');
    Ini.WriteString('Controller','Kp','0');
    Ini.WriteString('Controller','Ki','0');
    Ini.WriteString('Controller','Kd','0');
    Ini.WriteString('Motion','SpeedKmh',FloatToStr(GateSpeed,TFormatSettings.Invariant));
    Ini.WriteString('View','PixelsPerMetre','10');
    Ini.WriteString('View','OriginX','500');
    Ini.WriteString('View','OriginY','450');
    Ini.UpdateFile;
  finally Ini.Free; end;
end;

procedure CheckFrozen(const BeforeFrame, AfterFrame: TSimulationFrame; PIDFirst: Boolean);
var I: Integer;
begin
  if PIDFirst then
  begin
    Check(CompareMem(@BeforeFrame.Model,@AfterFrame.Model,SizeOf(BeforeFrame.Model)), 'PID model or target changed');
    Check(CompareMem(@BeforeFrame.Controller,@AfterFrame.Controller,SizeOf(BeforeFrame.Controller)), 'PID controller changed');
    Check(BeforeFrame.TrajectoryCount=AfterFrame.TrajectoryCount,'PID path length changed');
    for I:=0 to BeforeFrame.TrajectoryCount-1 do
    begin
      Near(BeforeFrame.Trajectory[I].X,AfterFrame.Trajectory[I].X,'PID path X changed');
      Near(BeforeFrame.Trajectory[I].Y,AfterFrame.Trajectory[I].Y,'PID path Y changed');
    end;
    Near(BeforeFrame.PIDResult.StopTime,AfterFrame.PIDResult.StopTime,'PID stop time changed');
    Near(BeforeFrame.PIDResult.MinimumDistance,AfterFrame.PIDResult.MinimumDistance,'PID result changed');
  end
  else
  begin
    Check(CompareMem(@BeforeFrame.Gate,@AfterFrame.Gate,SizeOf(BeforeFrame.Gate)), 'Gate state changed');
    Near(BeforeFrame.GateReferencePoint.X,AfterFrame.GateReferencePoint.X,'Gate target X changed');
    Near(BeforeFrame.GateReferencePoint.Y,AfterFrame.GateReferencePoint.Y,'Gate target Y changed');
    Check(BeforeFrame.GateTrajectoryCount=AfterFrame.GateTrajectoryCount,'Gate path length changed');
    for I:=0 to BeforeFrame.GateTrajectoryCount-1 do
    begin
      Near(BeforeFrame.GateTrajectory[I].X,AfterFrame.GateTrajectory[I].X,'Gate path X changed');
      Near(BeforeFrame.GateTrajectory[I].Y,AfterFrame.GateTrajectory[I].Y,'Gate path Y changed');
    end;
    Near(BeforeFrame.GateResult.StopTime,AfterFrame.GateResult.StopTime,'Gate stop time changed');
    Near(BeforeFrame.GateResult.MinimumDistance,AfterFrame.GateResult.MinimumDistance,'Gate result changed');
  end;
end;

procedure CheckOrder(GateSpeed: Double; PIDFirst: Boolean);
var F: TForm2; First, Last, Selected: TSimulationFrame; I: Integer; StoppedTime: Double; Png: TPngImage; Ini: TMemIniFile;
begin
  Configure(GateSpeed,20,10,True);
  if not PIDFirst then
  begin
    Ini:=TMemIniFile.Create(ChangeFileExt(ParamStr(0),'.ini'));
    try
      Ini.WriteString('ReferencePoint','X','100');
      Ini.WriteString('ReferencePoint','Y','100');
      Ini.WriteString('ReferencePoint','SpeedKmh','50');
      Ini.WriteString('ReferencePoint','HeadingDegrees','-10');
      Ini.WriteString('Controller','Kp','6');
      Ini.WriteString('Controller','Ki','10');
      Ini.WriteString('Controller','Kd','0.5');
      Ini.WriteString('Sensor','BiasDegrees','30');
      Ini.WriteString('Simulation','AutoStopDistance','20');
      Ini.UpdateFile;
    finally Ini.Free; end;
  end;
  F:=TForm2.Create(nil);
  try
    F.StartStopButtonClick(nil);
    for I:=1 to 3000 do
    begin
      F.Integrate(0.01);
      First:=F.DisplayFrame;
      if First.PIDResult.Finished or First.GateResult.Finished then Break;
    end;
    Check(First.PIDResult.Finished=PIDFirst,'Unexpected first algorithm');
    Check(First.GateResult.Finished<>PIDFirst,'Both algorithms stopped together');
    Check(F.IntegratorTimer.Enabled,'Timer stopped at first result');
    Check(not First.StoppedOnDistance,'First result marked complete');
    Check(Pos('min',F.ResultLabel.Caption)>0,'No intermediate result');
    // Manual pause and resume must not erase completion, minima or restart a finished algorithm.
    F.StartStopButtonClick(nil);
    Check(not F.IntegratorTimer.Enabled,'Manual pause');
    F.HistorySlider.Position:=0; F.HistorySliderChange(nil);
    F.StartStopButtonClick(nil);
    CheckFrozen(First,F.DisplayFrame,PIDFirst);
    for I:=1 to 3000 do
    begin
      F.Integrate(0.01);
      Last:=F.DisplayFrame;
      CheckFrozen(First,Last,PIDFirst);
      if not F.IntegratorTimer.Enabled then Break;
    end;
    Check(Last.PIDResult.Finished and Last.GateResult.Finished,'Second approach not finished');
    Check(Last.StoppedOnDistance and not F.IntegratorTimer.Enabled,'Final timer stop');
    Check(Abs(Last.Model.ReferencePoint.X-Last.GateReferencePoint.X)>0.01,'Target circles were not fixed independently');
    Near(Last.Model.ReferencePoint.X,Last.Model.ReferencePoint.InitialX + Last.Model.ReferencePoint.V * Cos(Last.Model.ReferencePoint.Fi) * Last.PIDResult.StopTime,'PID target freeze time');
    Near(Last.GateReferencePoint.X,Last.GateReferencePoint.InitialX + Last.GateReferencePoint.V * Cos(Last.GateReferencePoint.Fi) * Last.GateResult.StopTime,'Gate target freeze time');
    Check((Last.TrajectoryCount<=300) and (Last.GateTrajectoryCount<=300),'Path capacity');
    if PIDFirst then Check(Last.GateTrajectoryCount=300,'Long continuation not covered');
    Check(F.HistorySlider.Enabled,'Final history playback disabled');
    F.HistorySlider.Position:=0; F.HistorySliderChange(nil);
    Selected:=F.DisplayFrame;
    if Selected.Time >= First.Time then CheckFrozen(First,Selected,PIDFirst)
    else if PIDFirst then Check(not Selected.PIDResult.Finished,'Future PID result in history')
    else Check(not Selected.GateResult.Finished,'Future Gate result in history');
    F.HistorySlider.Position:=F.HistoryCount-1; F.HistorySliderChange(nil);
    Near(F.DisplayFrame.GateReferencePoint.X,Last.GateReferencePoint.X,'History target snapshot');
    StoppedTime:=F.SimulationTime;
    F.Integrate(0.01); F.IntegratorTimerTimer(nil); F.StartStopButtonClick(nil);
    Near(F.SimulationTime,StoppedTime,'Completed state advanced');
    Check(not F.IntegratorTimer.Enabled,'Completed run restarted without Init');
    Writeln('PASS order PID-first=',PIDFirst,'; PID t=',Last.PIDResult.StopTime:0:3,
      '; Gate t=',Last.GateResult.StopTime:0:3);
    if PIDFirst then
    begin
      Png:=TPngImage.Create;
      try Png.Assign(F.Image1.Picture.Bitmap); Png.SaveToFile(ChangeFileExt(ParamStr(0),'.png'));
      finally Png.Free; end;
    end;
    F.InitButtonClick(nil);
    Last:=F.DisplayFrame;
    Check(not Last.PIDResult.Finished and not Last.GateResult.Finished,'Init completion flags');
    Check((Last.TrajectoryCount=0) and (Last.GateTrajectoryCount=0),'Init path counts');
    Near(Last.Model.ReferencePoint.X,Last.Model.ReferencePoint.InitialX,'Init PID target');
    Near(Last.GateReferencePoint.X,Last.GateReferencePoint.InitialX,'Init Gate target');
    F.StartStopButtonClick(nil);
    Check(F.IntegratorTimer.Enabled,'Init cannot start');
  finally F.Free; end;
end;

procedure CheckModes;
var F: TForm2; I: Integer; Frame: TSimulationFrame;
begin
  Configure(150,-1,5,True);
  F:=TForm2.Create(nil);
  try
    F.StartStopButtonClick(nil); F.Integrate(0.01);
    Frame:=F.DisplayFrame;
    Check(Frame.PIDResult.Finished and Frame.GateResult.Finished,'Same-step completion');
    Check(not F.IntegratorTimer.Enabled,'Same-step timer');
  finally F.Free; end;
  Configure(150,-10,5,True);
  F:=TForm2.Create(nil);
  try
    F.Integrate(0.01);
    Frame:=F.DisplayFrame;
    Check(not Frame.PIDResult.Finished and not Frame.GateResult.Finished,'Stop outside threshold');
  finally F.Free; end;
  Configure(150,-1,0,True);
  F:=TForm2.Create(nil);
  try
    F.Integrate(0.01);
    Frame:=F.DisplayFrame;
    Check(not Frame.PIDResult.Finished and not Frame.GateResult.Finished,'Zero threshold');
  finally F.Free; end;
  Configure(72,20,5,False);
  F:=TForm2.Create(nil);
  try
    F.StartStopButtonClick(nil);
    for I:=1 to 200 do F.Integrate(0.01);
    Frame:=F.DisplayFrame;
    Check(not Frame.PIDResult.Finished and not Frame.GateResult.Finished,'Disabled auto stop');
    Check(F.IntegratorTimer.Enabled,'Disabled auto stop timer');
    Near(Frame.Model.ReferencePoint.X,Frame.GateReferencePoint.X,'Targets diverged before completion');
    F.StartStopButtonClick(nil);
    Check(not F.IntegratorTimer.Enabled,'Manual stop');
  finally F.Free; end;
end;
begin
  try
    Application.Initialize;
    CheckOrder(7.2,True);
    CheckOrder(150,False);
    CheckModes;
    DeleteFile(ChangeFileExt(ParamStr(0),'.ini'));
    Writeln('PASS: independent targets, frozen paths/controllers, history, pause/resume, reset, threshold, disabled stop');
  except on E: Exception do begin Writeln(E.ClassName,': ',E.Message); ExitCode:=1; end; end;
end.
