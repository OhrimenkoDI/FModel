program DistanceStopTest;
{$APPTYPE CONSOLE}
uses
  Winapi.Windows, System.SysUtils, System.Math, System.IniFiles, Vcl.Forms,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas',
  uPID in '..\uPID.pas';
// Проверяет порог на заведомо растущем расстоянии без влияния регулятора.
procedure CheckThreshold(StartX, Limit, Dt: Double; ShouldStop: Boolean);
var F: TForm2; Ini: TMemIniFile; Name: string;
begin
  Name := ChangeFileExt(ParamStr(0), '.ini');
  DeleteFile(Name);
  Ini := TMemIniFile.Create(Name);
  try
    Ini.WriteBool('Simulation', 'AutoStopEnabled', True);
    Ini.WriteString('Simulation', 'AutoStopDistance', FloatToStr(Limit, TFormatSettings.Invariant));
    Ini.WriteString('ReferencePoint', 'X', FloatToStr(StartX, TFormatSettings.Invariant));
    Ini.WriteString('ReferencePoint', 'Y', '0');
    Ini.WriteString('ReferencePoint', 'SpeedKmh', '0');
    Ini.WriteString('Controller', 'Kp', '0');
    Ini.WriteString('Controller', 'Ki', '0');
    Ini.WriteString('Controller', 'Kd', '0');
    Ini.UpdateFile;
  finally Ini.Free; end;
  F := TForm2.Create(nil);
  try
    if F.AutoStopDistance <> Limit then raise Exception.Create('Threshold INI load');
    F.StartStopButtonClick(nil);
    F.Integrate(Dt);
    if F.IntegratorTimer.Enabled = ShouldStop then
      raise Exception.CreateFmt('Threshold %g, start X %g: wrong stop state', [Limit, StartX]);
    if F.HistoryFrame(0).StoppedOnDistance <> ShouldStop then
      raise Exception.Create('Wrong stop flag in history');
  finally F.Free; end;
  DeleteFile(Name);
end;

var
  F: TForm2;
  Ini: TMemIniFile;
  Name: string;
  BeforeDistance, AfterDistance, StoppedTime, Minimum: Double;
  I: Integer;
begin
  try
    Application.Initialize;
    Name := ChangeFileExt(ParamStr(0), '.ini');
    Ini := TMemIniFile.Create(Name);
    try
      Ini.WriteBool('Simulation', 'AutoStopEnabled', True);
      Ini.WriteString('ReferencePoint', 'X', '1');
      Ini.WriteString('ReferencePoint', 'Y', '0');
      Ini.UpdateFile;
    finally Ini.Free; end;
    F := TForm2.Create(nil);
    try
      F.StartStopButtonClick(nil);
      Minimum := Hypot(F.ReferencePointX - F.Model.X, F.ReferencePointY - F.Model.Y);
      for I := 1 to 100 do
      begin
        BeforeDistance := Hypot(F.ReferencePointX - F.Model.X, F.ReferencePointY - F.Model.Y);
        F.Integrate(0.01);
        AfterDistance := Hypot(F.ReferencePointX - F.Model.X, F.ReferencePointY - F.Model.Y);
        Minimum := Min(Minimum, AfterDistance);
        if AfterDistance > BeforeDistance + 1E-9 then
        begin
          if F.IntegratorTimer.Enabled then raise Exception.Create('Did not stop on increase');
          Break;
        end;
        if not F.IntegratorTimer.Enabled then raise Exception.Create('Stopped while approaching');
      end;
      if F.IntegratorTimer.Enabled then raise Exception.Create('ReferencePoint was not passed');
      if not F.HistorySlider.Enabled then raise Exception.Create('No playback after auto stop');
      if not F.HistoryFrame(F.HistoryCount-1).StoppedOnDistance then
        raise Exception.Create('Final stop frame not recorded');
      if F.HistoryFrame(F.HistoryCount-1).Step <> F.StepCount then
        raise Exception.Create('Final history step mismatch');
      if Abs(F.MinimumDistance - Minimum) > 1E-10 then raise Exception.Create('Wrong minimum distance');
      if F.ResultLabel.Caption = '' then raise Exception.Create('Result not displayed');
      if F.MinimumDistance >= AfterDistance then raise Exception.Create('Displayed stop distance instead of minimum');
      StoppedTime := F.SimulationTime;
      F.IntegratorTimerTimer(nil);
      if F.SimulationTime <> StoppedTime then raise Exception.Create('Advanced after stop');
      F.StartStopButtonClick(nil);
      if F.ResultLabel.Caption <> '' then raise Exception.Create('Old result not cleared');
      if not F.IntegratorTimer.Enabled then raise Exception.Create('Cannot restart');
      F.InitButtonClick(nil);
      if F.SimulationTime <> 0 then raise Exception.Create('Reset failed');
      F.StartStopButtonClick(nil);
      Sleep(100);
      F.IntegratorTimerTimer(nil);
      if F.IntegratorTimer.Enabled then raise Exception.Create('Catch-up ignored automatic stop');
      if F.SimulationTime >= 0.1 then raise Exception.Create('Catch-up continued after automatic stop');
      StoppedTime := F.SimulationTime;
      F.IntegratorTimerTimer(nil);
      if F.SimulationTime <> StoppedTime then raise Exception.Create('Stopped backlog was integrated');
      F.AutoStopCheckBox.Checked := False;
      F.InitButtonClick(nil);
      if F.AutoStopEnabled then raise Exception.Create('Init changed auto stop setting');
      F.StartStopButtonClick(nil);
      for I := 1 to 100 do F.Integrate(0.01);
      if not F.IntegratorTimer.Enabled then raise Exception.Create('Disabled auto stop stopped calculation');
      if F.HistoryFrame(F.HistoryCount-1).StoppedOnDistance then
        raise Exception.Create('Disabled auto stop marked history as stopped');
      F.StartStopButtonClick(nil);
      if F.IntegratorTimer.Enabled then raise Exception.Create('Manual stop failed');
    finally F.Free; end;
    F := TForm2.Create(nil);
    try
      if F.AutoStopEnabled or F.AutoStopCheckBox.Checked then
        raise Exception.Create('Auto stop setting was not restored');
      F.AutoStopCheckBox.Checked := True;
      if not F.AutoStopEnabled then raise Exception.Create('Cannot enable auto stop');
    finally F.Free; end;
    DeleteFile(Name);
    CheckThreshold(-10, 5, 0.01, False);
    CheckThreshold(-1, 5, 0.01, True);
    CheckThreshold(-10, 20, 0.01, True);
    CheckThreshold(-4, 5, 0.024, False);
    CheckThreshold(-1, 0, 0.01, False);
    Writeln('PASS: distance threshold, strict boundary, INI, enabled/disabled stop, timer, reset');
  except
    on E: Exception do begin Writeln(E.Message); ExitCode := 1; end;
  end;
end.
