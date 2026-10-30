program ParameterDefaultsTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, Vcl.Forms, Vcl.ComCtrls, Vcl.StdCtrls,
 uParameters in '..\uParameters.pas', uAdaptiveGlide in '..\uAdaptiveGlide.pas';
var D:TParametersForm; Pages:TPageControl; E:TEdit; I,J:Integer;
 Values:TParameterValues; P:TAdaptiveParameters; A:array[0..15]of Double;
procedure Check(B:Boolean;const S:string);
begin if not B then raise Exception.Create(S); end;
begin
 try
  Application.Initialize; Values:=DefaultParameterValues; P:=DefaultAdaptiveParameters;
  A[0]:=P.CommandLimit; A[1]:=P.SlidingGain; A[2]:=P.SwitchingGain;
  A[3]:=P.Kappa; A[4]:=P.Lambda; A[5]:=P.IdentifierAlpha;
  A[6]:=P.CovarianceScale; A[7]:=P.DenominatorGuard; A[8]:=P.RateGain;
  A[9]:=P.AngleLimit;
  for I:=0 to 2 do begin A[10+I]:=P.ReferenceCoefficients[I]; A[13+I]:=P.IdentifierFilter[I]; end;
  for I:=0 to 15 do Check(Abs(A[I]-Values[36+I])<1E-12,'Controller/UI default mismatch');
  D:=TParametersForm.Create(nil);
  try
   Pages:=nil;
   for I:=0 to D.ComponentCount-1 do if D.Components[I] is TPageControl then Pages:=TPageControl(D.Components[I]);
   Check(Assigned(Pages),'Missing pages');
   for J:=0 to Pages.PageCount-1 do
   begin
    for I:=0 to High(Values) do TEdit(D.FindComponent('Parameter'+IntToStr(I))).Text:='custom-'+IntToStr(I);
    Pages.ActivePageIndex:=J;
    TButton(D.FindComponent('DefaultsButton')).Click;
    for I:=0 to High(Values) do
    begin
     E:=TEdit(D.FindComponent('Parameter'+IntToStr(I)));
     if TTabSheet(E.Parent.Parent).PageIndex=J then
      Check(E.Text=FloatToStr(Values[I]),'Active page not reset')
     else Check(E.Text='custom-'+IntToStr(I),'Other page changed');
    end;
    Check(D.ModalResult=0,'Reset applied/closed dialog');
   end;
  finally D.Free; end;
  Writeln('PASS: all 8 tabs reset independently; other text preserved; UI/core defaults match');
 except on E:Exception do begin Writeln(E.Message); ExitCode:=1; end; end;
end.
