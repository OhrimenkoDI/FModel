program RegulatorAudit;
{$APPTYPE CONSOLE}
uses
  System.SysUtils,
  uGuidanceController in '..\uGuidanceController.pas';
// Проверяет сохранение интегральной составляющей при нулевой ошибке.
procedure Audit(dt: Double);
var C: TGuidanceController; I: Integer;
begin
  C.Init;
  C.AngleCorrectionEnabled := False;
  C.Wint := 1;
  for I := 0 to Round(1 / dt)-1 do
  begin



    C.Update(dt, 0, C.W);
  end;
  if Abs(C.Wint - 1) > 1E-9 then raise Exception.Create('Integral grows at zero error');
  Writeln(Format('PASS: dt=%.3f; Wint=%.6f', [dt, C.Wint]));
end;
begin
  Audit(0.005); Audit(0.01); Audit(0.02);
end.
