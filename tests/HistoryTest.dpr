program HistoryTest;
{$APPTYPE CONSOLE}
uses
  System.SysUtils, Vcl.Forms, Vcl.Graphics, Vcl.Imaging.pngimage,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas',
  uGuidanceController in '..\uGuidanceController.pas';
// «авершает проверку с ошибкой, если условие не выполнено.
procedure Check(B: Boolean; const S: string);
begin
  if not B then raise Exception.Create(S);
end;
// ѕровер€ет совпадение двух чисел с допустимой погрешностью.
procedure Near(A,B: Double; const S: string);
begin
  Check(Abs(A-B)<1E-9,S);
end;
var
  F: TForm2;
  A,B,Displayed: TSimulationFrame;
  I: Integer;
  FinalX,FinalTime: Double;
  Bitmap: TBitmap;
  Png: TPngImage;
begin
  try
    Application.Initialize;
    F:=TForm2.Create(nil);
    try
      Check(F.HistoryCount=0,'Initial history');
      Check(not F.HistorySlider.Enabled,'Empty slider');
      for I:=1 to 220 do F.Integrate(0.005);
      Check(F.HistoryCount=200,'Capacity');
      A:=F.HistoryFrame(0); B:=F.HistoryFrame(199);
      Check((A.Step=21) and (B.Step=220),'Ring chronological order');
      Near(A.Time,0.105,'First retained time');
      Near(B.Time,1.1,'Last retained time');
      Check((A.TrajectoryCount=21) and (B.TrajectoryCount=220),'Historical trajectory');
      for I:=0 to 199 do
      begin
        Displayed:=F.HistoryFrame(I);
        Near(Displayed.Model.W,Displayed.Controller.Wprop+Displayed.Controller.Wint+
          Displayed.Controller.Wdiff,'PID terms sum');
        Near(Displayed.Controller.Wprop,Kp*Displayed.Controller.CorrectedAngle,'Proportional term');
      end;
      FinalX:=F.Model.X; FinalTime:=F.SimulationTime;
      F.HistorySlider.Position:=0;
      F.HistorySliderChange(nil);
      Displayed:=F.DisplayFrame;
      Near(Displayed.Model.X,A.Model.X,'Selected model');
      Near(Displayed.Time,A.Time,'Selected time');
      Near(F.Model.X,FinalX,'Scrub changed live model');
      Near(F.SimulationTime,FinalTime,'Scrub changed live clock');
      Check(Displayed.TrajectoryCount=21,'Future trajectory shown');
      F.HistorySlider.Position:=199;
      F.HistorySliderChange(nil);
      Displayed:=F.DisplayFrame;
      Near(Displayed.Model.X,FinalX,'Last frame');
      F.HistorySlider.Position:=50;
      F.HistorySliderChange(nil);
      Bitmap:=TBitmap.Create;
      Png:=TPngImage.Create;
      try
        Bitmap.SetSize(F.ClientWidth,F.ClientHeight);
        F.PaintTo(Bitmap.Canvas,0,0);
        Png.Assign(Bitmap);
        Png.SaveToFile('HistoryPreview.png');
      finally Png.Free; Bitmap.Free; end;
      F.StartStopButtonClick(nil);
      Check(not F.HistorySlider.Enabled,'Slider while running');
      Displayed:=F.DisplayFrame;
      Near(Displayed.Model.X,FinalX,'Resume rewound model');
      F.StartStopButtonClick(nil);
      Check(F.HistorySlider.Enabled,'Slider after manual stop');
      F.Integrate(0.005);
      B:=F.HistoryFrame(199);
      Check(B.Step=221,'Resume history continuity');
      F.InitButtonClick(nil);
      Check((F.HistoryCount=0) and not F.HistorySlider.Enabled,'Init clears history');
      F.Integrate(0);
      Check(F.HistoryCount=0,'Zero dt recorded');
    finally F.Free; end;
    Writeln('PASS: 200 frames, PID, chronological playback, trajectory, immutable live state, resume, reset');
  except on E: Exception do begin Writeln(E.ClassName+': '+E.Message); ExitCode:=1; end; end;
end.
