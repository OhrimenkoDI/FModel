program MotionModelTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, Vcl.Forms,
  uMain in '..\uMain.pas',
  uMotionModel in '..\uMotionModel.pas',
  uAdaptiveRegulator in '..\uAdaptiveRegulator.pas';
// Проверяет совпадение значений с допустимой погрешностью.
procedure Near(A,B: Double; const S: string);
begin
  if Abs(A-B)>1E-8 then raise Exception.Create(S);
end;
var
  M: TMotionModel;
  C: TAdaptiveRegulator;
  F: TForm2;
  I,Config: Integer;
  Dt,FirstAngle: Double;
begin
  try
    M.Init;
    for I:=1 to FAngleCount do
    begin
      Near(M.MeasureAngle,0,'Initial sensor buffer');
      M.Integrate(0.01);
      Near(M.W,0,'Plant changed external command');
    end;
    FirstAngle:=M.MeasureAngle;
    Near(FirstAngle,DegToRad(45+DefaultSensorBiasDegrees),'Sensor bias/delay');
    Near(M.Fi,0,'Uncontrolled model turned');
    M.W:=0.5;
    M.Integrate(0.02);
    Near(M.Fi,0.01,'External angular command');
    Application.Initialize;
    F:=TForm2.Create(nil);
    try
      for Config:=0 to 1 do
      begin
        F.InitButtonClick(nil);
        F.AngleCorrectionCheckBox.Checked:=Config=1;
        M:=F.Model;
        C:=F.Controller;
        for I:=0 to 120 do
        begin
          Dt:=0.007+(I mod 3)*0.001;
          // Независимая последовательность: измерение, регулятор, движение.
          M.W:=C.Update(Dt,M.MeasureAngle,M.W);
          M.Integrate(Dt);
          F.Integrate(Dt);
          Near(F.Model.X,M.X,'Main integration order X');
          Near(F.Model.Y,M.Y,'Main integration order Y');
          Near(F.Model.ReferencePoint.X,M.ReferencePoint.X,'Reference integration order X');
          Near(F.Model.ReferencePoint.Y,M.ReferencePoint.Y,'Reference integration order Y');
          Near(F.Model.W,C.W,'Command transmission');
          Near(F.Controller.Wint,C.Wint,'Controller state mismatch');
          Near(F.Controller.AdaptiveAngleCorrection,C.AdaptiveAngleCorrection,'Bias state mismatch');
        end;
      end;
      F.InitButtonClick(nil);
      F.Integrate(0);
      Near(F.StepCount,0,'Zero step mutated state');
      Near(F.Model.MeasuredAngle,0,'Zero step consumed sensor sample');
    finally F.Free; end;
    Writeln('PASS: independent plant, biased delayed sensor, explicit dt/angle controller, main ordering, replay');
  except on E: Exception do begin Writeln(E.Message); ExitCode:=1; end; end;
end.