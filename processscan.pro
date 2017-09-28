PRO processscan, MidAngles, ScanAvg, nScan, NearSunAngle, $
      RayleighOD, Rayleighgamma, pressure, SunElevMax, $      
      AngleShift, SideMatchFraction, MinAngle, MaxAngle, $
      FitParams, FitSigmas, FitMeasure, FitStatus, $
      DIAGPLOTFLAG = DiagPlotFlag, DIAGPLOTTEXT = DiagPlotText
      
IF N_ELEMENTS(DiagPlotFlag) LT 1 THEN DiagPlotFlag = 0
IF N_ELEMENTS(DIAGPLOTTEXT) LT 1 THEN DiagPlotText = ''
;
; September 2014 implement getting some parameters from a ULR scan
;
CheckShiftFlag = 0
shifttry = 0.5  ; degrees to try shifting data to match sides better
FracTolerance = 0.2  ;  note AERONET requires data on both sides to match within 0.2
MinAnglesForFit = 10
MinAngleRange = 30.  ; range of angles with valid data to do fit     (degrees)
SkyFraction = 0.000649565; of slit
IF SunELevMax LT 0.0 THEN BEGIN
  RayleighScale = 0.0
ENDIF ELSE BEGIN
  RayleighScale = RayleighOD*(pressure/1.01E5)/cos(((90.0-SunElevMax) < 89.9)*!DTOR)
  RayleighScale = 1.0 - exp(-RayleighScale)
  RayleighScale = RayleighScale*SkyFraction
ENDELSE
; Near Sun value
;
;aa = MIN(ABS(MidAngles - NearSunAngle), near1sub)
;aa = MIN(ABS(MidAngles + NearSunAngle), near2sub)
nangles = N_ELEMENTS(MidAngles)
halfn = FLOOR(nangles/2)
;
; pull out each side
;
side1 = REVERSE(ScanAvg[0:halfn-1])
side2 = ScanAvg[nangles - halfn:nangles - 1]
nside1 = REVERSE(nScan[0:halfn-1])
nside2 = nScan[nangles - halfn:nangles - 1]
angles = ABS(REVERSE(MidAngles[0:halfn-1]))
Tangles = angles
Tside1 = side1
SunValue = 0.5*(side1[0] + side2[0])

havedata = WHERE((nside1*nside2) GT 0 AND (angles GE NearSunAngle), nhavedata)
IF nhavedata LE MinAnglesForFit THEN BEGIN
  AngleShift = 0.0
  SideMatchFraction = 0.0
  MinAngle = 0.0
  MaxAngle= 0.0
  FitParams = [0.0, 0.0, 0.0, 0.0]
  FitSigmas = [0.0, 0.0, 0.0, 0.0]
  fitmeasure = 0.0
  FitStatus = -1
  RETURN
ENDIF
side1 = side1[havedata]
side2 = side2[havedata]
nside1 = nside1[havedata]
nside2 = nside2[havedata]
angles = angles[havedata]
side1 = side1[havedata]
side2 = side2[havedata]
;
; check small angle shifts
;
IF CheckShiftFlag THEN BEGIN
  side1shift = INTERPOL(side1, angles-shifttry, angles)
  side2shift = INTERPOL(side2, angles-shifttry, angles)
  side1shifth = INTERPOL(side1, angles-shifttry/2.0, angles)
  side2shifth = INTERPOL(side2, angles-shifttry/2.0, angles)
  noshiftcorr = CORRELATE(side1, side2)
  shift1corr = CORRELATE(side1shift, side2)
  shift2corr = CORRELATE(side1,side2shift)
  shift1corrh = CORRELATE(side1shifth, side2)
  shift2corrh = CORRELATE(side1,side2shifth)
  ;print, [noshiftcorr, shift1corr, shift2corr, shift1corrh, shift2corrh]
  MaxCorr = MAX([noshiftcorr, shift1corr, shift2corr, shift1corrh, shift2corrh], CorrIndx)
  CASE CorrIndx OF
    0: AngleShift = 0.0
    1: BEGIN
      AngleShift = shifttry
      side1 = side1shift
    END
    2: BEGIN
      AngleShift = -shifttry
      side2 = side2shift
    END
    3: BEGIN
      AngleShift = shifttry/2.0
      side1 = side1shifth
    END
    4: BEGIN
      AngleShift = -shifttry/2.0
      side2 = side2shifth
    END
  ENDCASE
ENDIF ELSE AngleShift = 0.0
;
; throw out mismatched data
;
SideAvg = 0.5*(side1 + side2)
SideRatio = side1/SideAvg
;plot, angles, sideratio, yrange = [0.7, 1.3]
;wait, 0.2
GData = WHERE((Sideratio LT (1.0+FracTolerance)) AND (SideRatio GT (1.0 - FracTolerance)), nGData)
SideMatchFraction = nGData/FLOAT(nhavedata)
IF nGData GT 1 THEN BEGIN
  SideAvg = SideAvg[GData]
  nSideAvg = nside1[GData] + nside2[GData]
  angles = angles[GData]
ENDIF
;weights = 1.0 + (angles GT 10.0) + (angles GT 30.0)  ; larger angles span greater range
weights = (1.0/SQRT(SideAvg > 1))*(1.0 + (angles GT 30.0))
MinAngle = Angles[0]
MaxAngle = angles[nGData-1]
anglerange = MaxAngle - MinAngle


IF (nGData GT MinAnglesForFit) AND (anglerange GT MinAngleRange) THEN BEGIN
  ;FitAngles = angles + (angles/ABS(angles))*2.8*(ABS(angles) > 2.0)^(-0.9425)
  FitAngles = angles + (angles/ABS(angles))*2.50*(ABS(angles) > 2.0)^(-0.9760)
  ; light through slit comes from range of angles
  ; on average this is a bigger angle than nominal angle. See slitangle.pro and IGOR SlitAngleFit
  ;  
  mus = COS(Fitangles*!DTOR)
  ;RayleighScat = RayleighScale*SunValue*(1.0435 + 0.9855*mus*mus)*(0.7289)
    ; 0.7289 to normalize the function 1 + mu^2 from zero to 1
  RayleighScat = RayleighScale*SunValue*((1.0 + 3.0*Rayleighgamma) + $
    (1.0 - Rayleighgamma)*mus*mus)*(0.75/(1.0 + 2.0*Rayleighgamma))
    ; Bucholtz eqtn as
  SideAvg = SideAvg - RayleighScat
  AvgSideAvg = TOTAL(SideAvg)/nGData
  ;
  ; fit to sum of Rayleigh and Henley-Greenstein
  ;
  ;IF MaxAngle GT 100.0 THEN BEGIN
  ;  FitParams = [10., 0.1*AvgSideAvg, 0.75, 0.5*AvgSideAvg]  ; initial guess
  ;  HGFitResult = CURVEFIT(mus, SideAvg, Weights, FitParams, FitSigmas, $
  ;     FUNCTION_NAME='hgrayfunction', CHISQ=fitmeasure, STATUS = fitstatus, ITMAX = 50, TOL = 0.01)
  ;    ; default 20 iterations max, 0.001
  ; ENDIF ELSE BEGIN
    TFitParams = [10., 0.1*AvgSideAvg, 0.75]  ; initial guess
    HGFitResult = CURVEFIT(mus, SideAvg, Weights, TFitParams, TFitSigmas, $
       FUNCTION_NAME='hgfunction', CHISQ=fitmeasure, STATUS = fitstatus, ITMAX = 50, TOL = 0.01)
      ; default 20 iterations max, 0.001
    FitParams = [TFitParams, RayleighScale*SunValue]
    FitSigmas = [TFitSigmas, RayleighScale*SunValue]
  ;ENDELSE
  IF DiagPlotFlag EQ 1 THEN BEGIN
    plot, fitangles, SideAvg, psym=4 ;, yrange = [0,800]
    oplot, fitangles, SideAvg + RayleighScat, psym=1
    oplot, fitangles, HGFitResult, color=2
    oplot, fitangles, RayleighScat, color=3
    XYOUTS, 0.6, 0.75, STRING(FitParams[2]), /NORM
    XYOUTS, 0.6, 0.7, STRING(RayleighOD), /NORM
    XYOUTS, 0.6, 0.8, DiagPlotText, /NORM
    SideAvg = SideAvg  ; there to allow a stop
  ENDIF
ENDIF ELSE BEGIN
  ; no fit
  FitParams = [0.0, 0.0, 0.0, RayleighScale*SunValue]
  FitSigmas = [0.0, 0.0, 0.0, RayleighScale*SunValue]
  fitmeasure = 0.0
  FitStatus = -1
ENDELSE

RETURN
END