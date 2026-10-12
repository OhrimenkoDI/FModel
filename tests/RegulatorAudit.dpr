program RegulatorAudit;
{$APPTYPE CONSOLE}
uses
  System.SysUtils,
  uPID in '..\uPID.pas';
// Проверяет сохранение интегральной составляющей при нулевой ошибке.
procedure Audit(dt: Double);
var C: TPIDState; I: Integer;
begin
  PIDinit(C);
  C.AngleCorrectionEnabled := False;
  C.Wint := 1;
  for I := 0 to Round(1 / dt)-1 do
  begin



    PIDupdate(C, dt, 0, C.W);
  end;
  if Abs(C.Wint - 1) > 1E-9 then raise Exception.Create('Integral grows at zero error');
  Writeln(Format('PASS: dt=%.3f; Wint=%.6f', [dt, C.Wint]));
end;
begin
  Audit(0.005); Audit(0.01); Audit(0.02);
end.
