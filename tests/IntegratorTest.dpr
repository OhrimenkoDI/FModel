program IntegratorTest;

{$APPTYPE CONSOLE}

uses
  Winapi.Windows, System.SysUtils, Vcl.Forms,
  uMain in '..\uMain.pas',
  uMotionModel in '..\uMotionModel.pas',
  uGuidanceController in '..\uGuidanceController.pas';

// Завершает проверку с ошибкой, если условие не выполнено.
procedure Check(Value: Boolean; const MessageText: string);
begin
  if not Value then
    raise Exception.Create(MessageText);
end;

var
  Form: TForm2;
  Started, Finished, Frequency: Int64;
  SavedTime, SavedX, SavedY: Double;
  SavedSteps: Int64;
begin
  try
    Application.Initialize;
    QueryPerformanceFrequency(Frequency);
    Form := TForm2.Create(nil);
    try
      Check(not Form.IntegratorTimer.Enabled, 'Must start stopped');
      Check(Form.SimulationTime = 0, 'Initial time');
      Check((Form.Model.X = 0) and (Form.Model.Y = 0) and
        (Form.Model.Fi = 0) and (Abs(Form.Model.V - DefaultSpeedKmh / 3.6) < 1E-10),
        'Initial model state');
      Form.StartStopButtonClick(nil);
      QueryPerformanceCounter(Started);
      Sleep(45);
      Form.IntegratorTimerTimer(nil);
      QueryPerformanceCounter(Finished);
      Check(Form.StepCount >= 4, 'Elapsed time was not split into substeps');
      Check(Form.Model.X > 0,
        'Timer must advance the motion model');
      Check((Form.LastDt > 0) and (Form.LastDt <= 0.0100000001), 'Substep too large');
      Check(Abs(Form.SimulationTime - (Finished - Started) / Frequency) < 0.02,
        'Substeps must preserve elapsed system time');
      Form.StartStopButtonClick(nil);
      SavedTime := Form.SimulationTime;
      SavedX := Form.Model.X;
      SavedY := Form.Model.Y;
      SavedSteps := Form.StepCount;
      Sleep(120);
      Form.IntegratorTimerTimer(nil);
      Check(Form.StepCount = SavedSteps, 'Stopped timer invoked integrator');
      Check(Form.SimulationTime = SavedTime, 'Time advanced while stopped');
      Check((Form.Model.X = SavedX) and (Form.Model.Y = SavedY),
        'Model moved while stopped');
      Form.StartStopButtonClick(nil);
      Form.IntegratorTimerTimer(nil);
      Check(Form.LastDt < 0.1, 'Resume included paused time');
      QueryPerformanceCounter(Started);
      repeat
        Application.ProcessMessages;
        Sleep(1);
        QueryPerformanceCounter(Finished);
      until (Finished - Started) / Frequency >= 0.3;
      Check(Form.StepCount > SavedSteps + 5, 'Timer did not dispatch periodic calls');
      Writeln(Format('Timer calls in ~0.3 s: %d; last dt: %.6f s',
        [Form.StepCount - SavedSteps - 1, Form.LastDt]));
      Form.InitButtonClick(nil);
      Check(not Form.IntegratorTimer.Enabled, 'Init must stop');
      Check((Form.Model.X = 0) and (Form.Model.Y = 0) and (Form.Model.Fi = 0) and
        (Abs(Form.Model.V - DefaultSpeedKmh / 3.6) < 1E-10), 'Init must reset model');
      Check((Form.SimulationTime = 0) and (Form.StepCount = 0) and
        (Form.LastDt = 0), 'Init must reset state');
      Form.StartStopButtonClick(nil);
      QueryPerformanceCounter(Started);
      Sleep(1200);
      Form.IntegratorTimerTimer(nil);
      Check(Form.StepCount = 100, 'Long backlog must be processed in bounded batches');
      Check(Form.HistoryCount = 100, 'Catch-up substeps missing from history');
      Check(Form.HistoryFrame(99).Step = 100, 'Catch-up history step number');
      Check(Form.SimulationTime <= 1.000000001, 'Catch-up batch exceeded 100 substeps');
      Form.IntegratorTimerTimer(nil);
      QueryPerformanceCounter(Finished);
      Check(Form.StepCount > 100, 'Unprocessed elapsed time was discarded');
      Check(Abs(Form.SimulationTime - (Finished - Started) / Frequency) < 0.05,
        'Catch-up did not preserve total elapsed time');
      Check(Form.LastDt <= 0.0100000001, 'Catch-up substep exceeded limit');
      Form.InitButtonClick(nil);
      Form.StartStopButtonClick(nil);
      // Destruction while running must disable the timer and release its resolution.
    finally
      Form.Free;
    end;
    Writeln('PASS: actual dt, start/stop, resume, timer dispatch, init, destruction');
  except
    on E: Exception do
    begin
      Writeln(E.ClassName + ': ' + E.Message);
      ExitCode := 1;
    end;
  end;
end.
