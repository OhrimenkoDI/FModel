program TrajectoryTest;
{$APPTYPE CONSOLE}
uses
  System.SysUtils, Vcl.Forms,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas',
  uAdaptiveRegulator in '..\uAdaptiveRegulator.pas';
var
  F: TForm2;
  I: Integer;
begin
  try
    Application.Initialize;
    F := TForm2.Create(nil);
    try
      if F.TrajectoryCount <> 0 then raise Exception.Create('Initial history');
      for I := 1 to TrajectoryCapacity + 50 do
        F.Integrate(0.01);
      if F.TrajectoryCount <> TrajectoryCapacity then raise Exception.Create('History capacity');
      F.FormResize(F);
      F.IntegratorTimerTimer(nil);
      if F.TrajectoryCount <> TrajectoryCapacity then raise Exception.Create('Paused history');
      F.InitButtonClick(nil);
      if F.TrajectoryCount <> 0 then raise Exception.Create('Reset history');
      F.Integrate(0);
      if F.TrajectoryCount <> 0 then raise Exception.Create('Zero dt history');
      F.Integrate(0.01);
      if F.TrajectoryCount <> 1 then raise Exception.Create('First measurement');
    finally F.Free; end;
    Writeln('PASS: Trajectory capacity, redraw, pause, reset, recording');
  except
    on E: Exception do begin Writeln(E.Message); ExitCode := 1; end;
  end;
end.
