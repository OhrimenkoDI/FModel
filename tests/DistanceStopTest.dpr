program DistanceStopTest;
{$APPTYPE CONSOLE}
uses
  Winapi.Windows, System.SysUtils, System.Math, System.IniFiles, Vcl.Forms,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas',
  uGuidanceController in '..\uGuidanceController.pas';
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
      Ini.WriteString('Target', 'X', '1');
      Ini.WriteString('Target', 'Y', '0');
      Ini.UpdateFile;
    finally Ini.Free; end;
    F := TForm2.Create(nil);
    try
      F.StartStopButtonClick(nil);
      Minimum := Hypot(F.TargetX - F.Model.X, F.TargetY - F.Model.Y);
      for I := 1 to 100 do
      begin
        BeforeDistance := Hypot(F.TargetX - F.Model.X, F.TargetY - F.Model.Y);
        F.Integrate(0.01);
        AfterDistance := Hypot(F.TargetX - F.Model.X, F.TargetY - F.Model.Y);
        Minimum := Min(Minimum, AfterDistance);
        if AfterDistance > BeforeDistance + 1E-9 then
        begin
          if F.IntegratorTimer.Enabled then raise Exception.Create('Did not stop on increase');
          Break;
        end;
        if not F.IntegratorTimer.Enabled then raise Exception.Create('Stopped while approaching');
      end;
      if F.IntegratorTimer.Enabled then raise Exception.Create('Target was not passed');
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
    finally F.Free; end;
    DeleteFile(Name);
    Writeln('PASS: approach, first increase, stopped timer, restart and reset');
  except
    on E: Exception do begin Writeln(E.Message); ExitCode := 1; end;
  end;
end.
