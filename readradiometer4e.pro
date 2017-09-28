pro readradiometer4e, filename, skipstart, $
  WAVENAMESUFFIX = wavenamesuffix, $
  UNITNUMBER = UnitNumber, $
  READFROMMERGE = ReadFromMerge, $
  INPATH = InPath
;
; 20160409 version even less inline code
; 20140917 version with less inline code 
; 
; 20141224 version with barometer, more housekeeping including gates, unit number call option 
;
IF N_PARAMS(0) LT 2 THEN skipstart = 2000
IF N_ELEMENTS(WaveNameSuffix) LT 1 THEN wavenamesuffix = ''
IF N_ELEMENTS(UNITNUMBER) LT 1 THEN UnitNumber = 0
IF N_ELEMENTS(READFROMMERGE) LT 1 THEN ReadFromMerge = 0
;
;IF N_ELEMENTS(INPATH) LT 1 THEN inpath = '/Volumes/Data HD2/Radiom/'
IF N_ELEMENTS(INPATH) LT 1 THEN inpath = '/Volumes/Data HD/Radiom/SASP_testruns/'
;inpath = '/Volumes/Macintosh HD/Data/'
  
;
; constants 
;
Wavelengths = [550.4, 460.3, 671.2, 860.7]   ; decimals from curves on filters - same batch?
RayleighOD = [0.0972023, 0.202482, 0.0435789, 0.0158781] ; at sea level zenith
Rayleighgamma = [1.471, 1.442, 1.413, 1.384]; Bucholtz
Rayleighgamma = 0.01*Rayleighgamma
; from estimateRayleigh.pro

NearSunAngle = 4.32     ;  angle for "near sun" value to subtract sky light (degrees to each side) typically 4 to 5
NearSunFitAngle = 6.33 ;  closest point of fitting scattered light to asymmetry parameter (degrees to each side) typically 6 to 9
  ; helpful if it matches a midpoint of AvgAngleLo, Hi below     
MaxAvgDegrees = 2.2      ; half width  Average n highest values within this of middle of peak to get max 
nforMaxAvg = 5           ; about 0.35 degree per value    
 
; averaging bins for scattering angle
AvgAngleWidths = [REPLICATE(2.5,60),REPLICATE(1.0,20),REPLICATE(2./3.,30),REPLICATE(1.0,20),REPLICATE(2.5,60)]
AvgAngleHi = -180 + TOTAL(AvgAngleWidths,/CUM)
AvgAngleLo = AvgAngleHi - AvgAngleWidths
MidAngles = 0.5*(AvgAngleLo + AvgAngleHi)
nAvgAngles = N_ELEMENTS(AvgAngleLo)
;
; -------------------------------------- Read in Data -----------------------------------------
;
IF ReadFromMerge EQ 0 THEN BEGIN
  readradiometerfile, filename, skipstart, unitnumber, $
    OutFileName, rotrate, shortgate, longgate, npoints, nhk,  $  ; scalar outputs
    PhotoAMerge, PhotoBMerge, PhotoCMerge, PhotoDMerge, $        ; array outputs
    AzimuthA, AzimuthB, AzimuthC, AzimuthD, $
    UDT,  Homep, $                                             ; other outputs each data point
    HKTime, te, lat, lon, Month, Day, Year, $                    ; outputs each housekeeping point (fewer points)
    Yaw, Pitch, Roll, BaromPr, BaromTe, SolarZenithsh, SolarAzimuthsh, UDTforZenith, $
    ModeFlag, HKPhotoMax, HKPhotoAvg, $
    INPATH = InPath, $
    SUFFIX = Suffix, $   ; this for input file
    WRITEFILEFLAG = WriteFileFlag, OUTPUTPATH = OutputPath, $
    WAVENAMESUFFIX = WaveNameSuffix  ; for output files

ENDIF ELSE BEGIN
  ; read from already processed file
ENDELSE
;
; -------------------------------------- FIND SUN PEAKS -----------------------------------------
;
; find peaks in signal indicating sun 
;   useforThreshold is master but then find same peaks in other channels
;   set at "C" for now: 860 is cleanest wavelength but position can be a little off - different optics
;
UseForThreshold = 'C'
CASE STRUPCASE(UseForThreshold) OF
  'A': SunPhoto = PhotoAMerge
  'B': SunPhoto = PhotoBMerge
  'C': SunPhoto = PhotoDMerge
  'D': SunPhoto = PhotoDMerge
  ELSE: STOP
ENDCASE
histlnSunPhoto = HISTOGRAM(ALOG(SunPhoto > 10.), nbins = 200, LOCATIONS = HistBins)
CumHist = TOTAL(REVERSE(histlnSunPhoto), /CUM)
SunPhoto95 = WHERE(CumHist GE 0.05*N_ELEMENTS(SunPhoto), nOver95)
;  slit is 3 degrees so about 1% of data is very high, go to 95% for not all peaks being high, etc.
PercentilePt = EXP(HistBins[nOver95[0]])
SunThresholdMerge = PercentilePt


FindSunPeaks, SunPhoto, UDT, SunThresholdMerge, $
  npeaks, PeakUDT, LoIndices, HiIndices, PeakTypeFlags, $
  ROTRATE = rotrate, TIMESPACING = shortgate + longgate, $
  MaxWidthAngle = 9., ClosestAbove = 30., GapAngle = 600. ; all in degrees 
MidIndices = ROUND(0.5*(LoIndices + HiIndices))
;
; -------------------------------------- CALCULATIONS FOR EACH MAXIMUM ---------------------------------------
;
;
; information about maxima such as time, lat, lon, sun elevation
;
ishort = N_ELEMENTS(HKTime)
IF npeaks GT 0 THEN BEGIN
  lastUDTofHK = HKTime[ishort-1]-0.0001
  LatMax = INTERPOL(lat, HKTime, (PeakUDT < lastUDTofHK))
  LonMax = INTERPOL(lon, HKTime, (PeakUDT < lastUDTofHK))
  PrMax =  INTERPOL(Barompr, HKTime, (PeakUDT < lastUDTofHK))

  YawX = SIN(Yaw*!DTOR)
  YawY = COS(Yaw*!DTOR)
  UDTforInterp  = ((UDT < lastUDTofHK) > HKTime[0])
  YawXEvery = INTERPOL(YawX, HKTime, UDTforInterp)
  YawYEvery = INTERPOL(YawY, HKTime, UDTforInterp)
  YawEvery = ATAN(YawXEvery,YawYEvery)/!DTOR
  YawMax = YawEvery[midindices]
 
  SolarZenith = INTERPOL(SolarZenithsh, UDTforZenith, UDTforInterp)
  SolarAzimuth = INTERPOL(SolarAzimuthsh, UDTforZenith, UDTforInterp)
  SunElevMax = 90.0 - SolarZenith[MidIndices]
  SunAzMax = SolarAzimuth[MidIndices] 
  LoScanIndices = LONARR(npeaks)  ; range of indices for scan centered on peak (LoIndices is range of peak) 
  HiScanIndices = LoScanIndices
  SunMax = FLTARR(4, npeaks)
  SunAvgMax = SunMax
  NearSun = SunMax
  AvgPhoto = FLTARR(4, npeaks, NAvgAngles)
  AvgScatAngle =  FLTARR(npeaks, NAvgAngles)
  AvgRelazAngle = AvgScatAngle  ; within scatangle
  AvgAzimuth = AvgScatAngle
  AvgAzimuthA = AvgScatAngle
  AvgAzimuthB = AvgScatAngle
  AvgAzimuthC = AvgScatAngle
  AvgAzimuthD = AvgScatAngle
 nAvgAngle = INTARR(npeaks, NAvgAngles)
  ;
  ; relative azimuth, interpolated sky radiance in sun, 
  ;
  relazimuth = SolarAzimuth*0+500. ; sizes array and fills with value larger than 180 degrees.
  scatangle = relazimuth
  n180 = CEIL(180./(rotrate*(shortgate + longgate)))
  nlook = 2*n180+1
  aa = MIN(ABS(MidAngles - NearSunAngle), near1sub)
  aa = MIN(ABS(MidAngles + NearSunAngle), near2sub)
  FOR ipk = 0, npeaks-1 DO BEGIN
    loindx = (midindices[ipk] - n180) > 0
    LoScanIndices[ipk] = loindx
    theseindices = loindx+  LINDGEN(nlook < (npoints-(loindx+1))) 
    localyaw = YawEvery[theseindices] - YawEvery[midindices[ipk]]
    localrelaz = (UDT[theseindices] - PeakUDT[ipk])*(3600.*rotrate)
    prevrelaz = relazimuth[theseindices]  ; fix nans
    localrelaz = localrelaz ;+ localyaw
    ;smaller = WHERE(ABS(localrelaz) LT ABS(prevrelaz) , nsmaller)
    smaller = WHERE(ABS(localrelaz) LT (ABS(prevrelaz) < 180.) , nsmaller)
    IF nsmaller GT 0 THEN BEGIN
      theserelaz = localrelaz[smaller]
      arrayindices = theseindices[smaller]
      relazimuth[arrayindices] = theserelaz
      coszenith = sin(SunElevMax[ipk]*(!DTOR*1.0D))   ; sin and cos reversed for elevation versus zenith
      sinzenith = cos(SunElevMax[ipk]*(!DTOR*1.0D))
      signangle = 2.0*(theserelaz GE 0.0) - 1.0
      ; equation 2 Box and Deepak 1981:
      thesescatangle = (signangle/!DTOR)*ACOS(coszenith*coszenith + sinzenith*sinzenith*cos(theserelaz*!DTOR))
      scatangle[arrayindices] = thesescatangle
      ;
      ; maxima: average a few points or just the maximum value
      ;
      formax = WHERE(ABS(theserelaz) LT MaxAvgDegrees,nformax)
      ;if ipk GT 75 THEN STOP
      IF nformax GT 0 THEN indicesformax = arrayindices[formax]
      IF nformax GE nforMaxAvg THEN BEGIN
        sorti = REVERSE(SORT(PhotoAMerge[indicesformax]))
        SunAvgMax[0, ipk] = TOTAL(PhotoAMerge[indicesformax[sorti[0:nforMaxAvg-1]]])/nforMaxAvg
        sorti = REVERSE(SORT(PhotoBMerge[indicesformax]))
        SunAvgMax[1, ipk] = TOTAL(PhotoBMerge[indicesformax[sorti[0:nforMaxAvg-1]]])/nforMaxAvg
        sorti = REVERSE(SORT(PhotoCmerge[indicesformax]))
        SunAvgMax[2, ipk] = TOTAL(PhotoCMerge[indicesformax[sorti[0:nforMaxAvg-1]]])/nforMaxAvg
        sorti = REVERSE(SORT(PhotoDMerge[indicesformax]))
        SunAvgMax[3, ipk] = TOTAL(PhotoDMerge[indicesformax[sorti[0:nforMaxAvg-1]]])/nforMaxAvg
      ENDIF
      ;IF SunAvgMax[1,ipk] EQ 0 THEN STOP
      IF nformax GT 0 THEN BEGIN
        SunMax[0, ipk] = MAX(PhotoAMerge[indicesformax])
        SunMax[1, ipk] = MAX(PhotoBMerge[indicesformax])
        SunMax[2, ipk] = MAX(PhotoCMerge[indicesformax])
        SunMax[3, ipk] = MAX(PhotoDMerge[indicesformax])
      ENDIF
      ;
      ; average values over a range of scattering angles
      ;
      FOR iangle = 0, nAvgAngles-1 DO BEGIN
        InRange = WHERE((thesescatangle GE AvgAngleLo[iangle]) AND (thesescatangle LT AvgAngleHi[iangle]), nInRange)
        IF nInRange GT 0 THEN BEGIN
          indicesforaverage = arrayindices[InRange]
          AvgPhoto[0, ipk, iangle] = TOTAL(PhotoAMerge[indicesforaverage])/nInRange
          AvgPhoto[1, ipk, iangle] = TOTAL(PhotoBMerge[indicesforaverage])/nInRange
          AvgPhoto[2, ipk, iangle] = TOTAL(PhotoCMerge[indicesforaverage])/nInRange
          AvgPhoto[3, ipk, iangle] = TOTAL(PhotoDMerge[indicesforaverage])/nInRange
 ;                IF avgphoto[0, ipk, 145] GT avgphoto[0, ipk, 110] THEN STOP
          if (ipk EQ 2000) AND (iangle EQ nAvgAngles-2)  THEN STOP  
          if (ipk EQ 2001) AND (iangle EQ nAvgAngles-2)  THEN STOP  
          AvgRelazAngle[ipk, iangle] = TOTAL(theserelaz[InRange])/nInRange
          AvgAzimuthA[ipk, iangle] = TOTAL(azimuthA[indicesforaverage])/nInRange
          AvgAzimuthB[ipk, iangle] = TOTAL(azimuthB[indicesforaverage])/nInRange
          AvgAzimuthC[ipk, iangle] = TOTAL(azimuthC[indicesforaverage])/nInRange
          AvgAzimuthD[ipk, iangle] = TOTAL(azimuthD[indicesforaverage])/nInRange
          nAvgAngle[ipk, iangle] = nInRange  
          AvgScatAngle[ipk, iangle] =   TOTAL(thesescatangle[InRange])/nInRange   
        ENDIF  
      ENDFOR
    ENDIF
  ENDFOR
  NearSun = 0.5*(AvgPhoto[*,*,near1sub] + AvgPhoto[*,*,near2sub])
  notclose = WHERE(ABS(relazimuth) GT 200., nnotclose)
  if nnotclose GT 0 THEN BEGIN
    relazimuth[notclose] = !VALUES.F_NAN
    scatangle[notclose] = !VALUES.F_NAN
  ENDIF
   plot, 1/cos(90.-sunelevmax*!DTOR), SunMax[0,*], xrange = [0, 6]
ENDIF ELSE BEGIN  ; had maxima
  relazimuth = azimuth*0.  ; sizes
  scatangle = relazimuth
ENDELSE
;
; ---------------------------------- MORE CALCULATIONS USING MAXIMA ---------------------------------------
;
IF nPeaks GT 0 THEN BEGIN
  PeakFitParamNames = ['AngleShift', 'SideMatchFrac', 'MinAngle', 'MaxAngle', $
      'FitConst','HGamp','HGg','Rayamp','HGampSig','FitConstSig','HGgsig','Rayampsig','FitMeasure','FitStatus']
  nPeakParams = N_ELEMENTS(PeakFitParamNames)
  PeakFitParams = FLTARR(4, nPeakParams, npeaks)
  MidAngles = 0.5*(AvgAngleHi + AvgAngleLo)  
  FOR ipk = 0, npeaks-1 DO BEGIN
    thesen = REFORM(nAvgAngle[ipk, *])
    TheseAngles = REFORM(AvgScatAngle[ipk, *])
    FOR icolor = 0, 3 DO BEGIN
      ProcessScan, TheseAngles, REFORM(AvgPhoto[icolor, ipk, *]), thesen,  NearSunFitAngle, $
        RayleighOD[icolor], Rayleighgamma[icolor], PrMax[ipk], SunElevMax[ipk], $        
        AngleShift, SideMatchFraction, MinAngle, MaxAngle, $
        FitParams, FitSigmas, FitMeasure, FitStatus, $
        DIAGPLOTTEXT = STRING(Wavelengths[icolor])+STRING(ipk) , $
        DIAGPLOTFLAG = (ipk MOD 200) EQ 2
 ;       DIAGPLOTFLAG = ((ipk MOD 200) EQ 2 AND (icolor EQ 1))
      PeakFitParams[icolor, 0, ipk] = AngleShift
      PeakFitParams[icolor, 1, ipk] = SideMatchFraction
      PeakFitParams[icolor, 2, ipk] = MinAngle
      PeakFitParams[icolor, 3, ipk] = MaxAngle
      PeakFitParams[icolor, 4:7, ipk] = FitParams
      PeakFitParams[icolor, 8:11, ipk] = FitSigmas
      PeakFitParams[icolor, 12, ipk] = FitMeasure
      PeakFitParams[icolor, 13, ipk] = FitStatus
    ENDFOR
  ENDFOR
ENDIF

;
;  ---------------------------------- OUTPUT ---------------------------------------
;
;
; output main data to IGOR text files so more than one variable per file
;  several files: time series data (A), housekeeping (H), peaks (M for max). more peak stuff (MA)
;    and angle-averaged (binary file?)
;
nfilesplit = 1  ; not sure whether to split long, long files here, in IGOR, or in textwrangler
print, 'npeaks means'
print, npeaks, mean(photoAMerge), mean(photoBMerge), mean(photoCMerge), mean(photoDMerge)
tratio = shortgate/longgate
print, udt[0], udt[-1];, max(photoAmerge)*tratio, max(photoBsh)*tratio, max(photoCsh)*tratio, max(photoDsh)*tratio
if npeaks GT 1 THEN BEGIN
  FOR i = 0, nfilesplit-1 DO BEGIN
    ;
    ; maxima
    ;
    OPENW, OutFile, inpath + filename + 'M'+'.itx', /GET_LUN
    PRINTF, OutFile, 'IGOR'
    namestring = 'WAVES /O UDTMax, SunElevMax, SunAzMax, SunMaxA, SunMaxB, SunMaxC, SunMaxD, '
    namestring = namestring + 'SunAvgMaxA, SunAvgMaxB, SunAvgMaxC, SunAvgMaxD, NearSunA, NearSunB, NearSunC, NearSunD, PrMax'
    PRINTF, OutFile, namestring
    PRINTF, OutFile, 'BEGIN'
    FOR iout = 0L, npeaks-1 DO BEGIN
      PRINTF, OutFile, PeakUDT[iout], SunElevMax[iout], SunAzMax[iout], $
        SunMax[0, iout], SunMax[1, iout], SunMax[2, iout], SunMax[3, iout], $
        Sunavgmax[0, iout], Sunavgmax[1, iout], Sunavgmax[2, iout], Sunavgmax[3, iout], $
        NearSun[0, iout], NearSun[1, iout], NearSun[2, iout], NearSun[3, iout], prmax[iout]
    ENDFOR
    PRINTF, OutFile, 'END'
    PRINTF, OutFile, 'X Duplicate /O sunelevmax Langleyxmax lnAmax, lnBmax, lnCmax, lnDmax'
    PRINTF, OutFile, 'X lnAmax = ln(SunAvgMaxA - NearSunA)
    PRINTF, OutFile, 'X lnBmax = ln(SunAvgMaxB - NearSunB)
    PRINTF, OutFile, 'X lnCmax = ln(SunAvgMaxC - NearSunC)
    PRINTF, OutFile, 'X lnDmax = ln(SunAvgMaxD - NearSunD)
    PRINTF, OutFile, 'X langleyxmax = 1.0/cos((90.0 - sunelevmax)*pi/180.)
    CLOSE, OutFile
    ; -------------- more peak stuff ---------------------
    OPENW, OutFile, inpath + filename + 'MA'+'.itx'
    PRINTF, OutFile, 'IGOR'
    namestring = 'WAVES /O LatMax, LonMax, LoIndicesMax, HiIndicesMax, MaxAngleA, MaxAngleB, MaxAngleC, MaxAngleD, YawMax'
    PRINTF, OutFile, namestring
    PRINTF, OutFile, 'BEGIN'
    FOR iout = 0L, npeaks-1 DO BEGIN
      PRINTF, OutFile, LatMax[iout], LoIndices[iout], HiIndices[iout], $
       PeakFitParams[0, 3, iout], PeakFitParams[1, 3, iout], PeakFitParams[2, 3, iout], PeakFitParams[3, 3, iout], $
       YawMax[iout]
    ENDFOR
    PRINTF, OutFile, 'END'
    namestring = 'WAVES /O ConstA, ConstB, ConstC, ConstD, '
    namestring = namestring + 'HGampA, HGampB, HGampC, HGampD, HGgA, HGgB, HGgC, HGgD, RayA, RayB, RayC, RayD'
    PRINTF, OutFile, namestring
    PRINTF, OutFile, 'BEGIN'
    FOR iout = 0L, npeaks-1 DO BEGIN
      PRINTF, OutFile, $
       PeakFitParams[0, 4, iout], PeakFitParams[1, 4, iout], PeakFitParams[2, 4, iout], PeakFitParams[3, 4, iout], $
       PeakFitParams[0, 5, iout], PeakFitParams[1, 5, iout], PeakFitParams[2, 5, iout], PeakFitParams[3, 5, iout], $
       PeakFitParams[0, 6, iout], PeakFitParams[1, 6, iout], PeakFitParams[2, 6, iout], PeakFitParams[3, 6, iout], $
       PeakFitParams[0, 7, iout], PeakFitParams[1, 7, iout], PeakFitParams[2, 7, iout], PeakFitParams[3, 7, iout]
    ENDFOR
    PRINTF, OutFile, 'END'
    FREE_LUN, OutFile
    ;
    ; big arrays of data over scan angle
    ;
    WriteIgorBinary, WroteIt, AvgPhoto, filename +'AvgPhoto',  FILEPATH = inpath
    WriteIgorBinary, WroteIt, AvgScatAngle, filename +'AvgScatAngl',  FILEPATH = inpath
    ;WriteIgorBinary, WroteIt, AvgRelazAngle, filename +'AvgRelazAngle',  FILEPATH = inpath
    ;WriteIgorBinary, WroteIt, AvgAzimuth, filename +'AvgAzimuth',  FILEPATH = inpath
    WriteIgorBinary, WroteIt, nAvgAngle, filename +'nAvgPhoto',  FILEPATH = inpath
    WriteIgorBinary, WroteIt, MidAngles, filename +'MidAngles',  FILEPATH = inpath
  ENDFOR
ENDIF


return
end
; adjust azimuth about 2 to 2.5 degrees in Arduino
; more digits on millisGPS output in Arduino