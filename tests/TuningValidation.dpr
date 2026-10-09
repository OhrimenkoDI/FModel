program TuningValidation;
{$APPTYPE CONSOLE}
uses
  System.SysUtils, System.Math,
  uMotionModel in '..\uMotionModel.pas',
  uGuidanceController in '..\uGuidanceController.pas';
const
  Targets: array[0..1, 0..1] of Double =
    ((94.6605734183323, 36.4706067517867), (100, 100));
  Steps: array[0..6, 0..3] of Double =
    ((0.01, 0.01, 0.01, 0.01), (0.015625, 0.015625, 0.015625, 0.015625),
     (0.02, 0.02, 0.02, 0.02), (0.009, 0.018, 0.012, 0.016),
     (0.012, 0.012, 0.012, 0.012), (0.018, 0.018, 0.018, 0.018),
     (0.011, 0.017, 0.013, 0.019));
var
  M: TMotionModel;
  Controller: TGuidanceController;
  T, S, I: Integer;
  BeforeDistance, Distance, Nearest: Double;
  Stopped: Boolean;
begin
  try
    for T := 0 to High(Targets) do
      for S := 0 to High(Steps) do
      begin
        M.Init;
        Controller.Init;
        M.Target.X := Targets[T, 0];
        M.Target.Y := Targets[T, 1];
        Distance := Hypot(M.Target.X, M.Target.Y);
        Nearest := Distance;
        Stopped := False;
        for I := 0 to 1999 do
        begin
          BeforeDistance := Distance;
          M.W := Controller.Update(Steps[S, I mod 4], M.MeasureAngle, M.W);
          M.Integrate(Steps[S, I mod 4]);
          Distance := Hypot(M.Target.X - M.X, M.Target.Y - M.Y);
          Nearest := Min(Nearest, Distance);
          if Distance > BeforeDistance + 1E-9 then
          begin
            Stopped := True;
            Break;
          end;
        end;
        if not Stopped or IsNan(Distance) or IsInfinite(Distance) then
          raise Exception.Create('Simulation did not finish with finite distance');
        if Abs(M.V - DefaultSpeedKmh * KmhToMetresPerSecond) > 1E-10 then
          raise Exception.Create('Speed changed');
        Writeln(Format('Target %d, timing %d: final %.6f m; nearest sample %.6f m',
          [T, S, Distance, Nearest]));
      end;
    Writeln('PASS: 14 simulation cases, fixed speed, sensor-only controller; no true sensor parameters passed');
  except
    on E: Exception do begin Writeln(E.Message); ExitCode := 1; end;
  end;
end.
