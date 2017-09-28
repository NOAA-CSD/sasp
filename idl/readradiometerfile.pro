pro readradiometerfile, filename, skipstart, unitnumber, $
  OutFileName, rotrate, shortgate, longgate, npoints, nhk,  $  ; scalar outputs
  PhotoAMerge, PhotoBMerge, PhotoCMerge, PhotoDMerge, $        ; array outputs
  AzimuthA, AzimuthB, AzimuthC, AzimuthD, $
  UDT,  Homep, $                                             ; other outputs each data point
  UDTofHK, te, lat, lon, Month, Day, Year, $                    ; outputs each housekeeping point (fewer points)
  Yaw, Pitch, Roll, BaromPr, BaromTe, SolarZenithsh, SolarAzimuthsh, UDTforZenith, $
  ModeFlag, HKPhotoMax, HKPhotoAvg, $
  INPATH = InPath, $
  SUFFIX = Suffix, $   ; this for input file
  WRITEFILEFLAG = WriteFileFlag, OUTPUTPATH = OutputPath, $
  WAVENAMESUFFIX = WaveNameSuffix, $
  FIXAZIMUTHFLAG = FixAzimuthFlag  ; for output files
;
; April 2016 split out reading file from rest of code
;   known issues: 
;     - will fail if there are no data points within reasonable range (positive but less than 2^20)
;     - may fail if file crosses into new year, not debugged for new month
;     - often valid GPS may not be present at start of file. Filling this in may fail across midnight
;       (don't turn on instrument a minute before midnight UDT)
;       (workaround is to skip start of file with skipstart variable)
;
IF N_ELEMENTS(INPATH) EQ 0 THEN InPath = '/Volumes/Data HD/Radiom/'
IF N_ELEMENTS(SUFFIX) EQ 0 THEN Suffix = '.txt'
IF N_ELEMENTS(FileType) EQ 0 THEN FileType = 1
IF N_ELEMENTS(WRITEFILEFLAG) EQ 0 THEN WriteFileFlag = 1
IF N_ELEMENTS(OUTPUTPATH) EQ 0 THEN OutputPath = inpath
IF N_ELEMENTS(WAVENAMESUFFIX) EQ 0 THEN WaveNameSuffix = ''
IF N_ELEMENTS(FIXAZIMUTHFLAG) EQ 0 THEN FixAzimuthFlag = 0

;
;  constants for reading and interpreting file
;
GateType = 0  ; read from file
rotationrate = 5194.8   ; ms per radian, see unit number statements below
millisscale = 10.        ; factor in Arduino print statement
;
;
nlong = 8000000L    ; maximum number of lines in a SASP file
; 9 hours is about a million points
nshort = nlong/15L
HiThres = 10000L     ; when to transition from long to short gate (raw units)  ; 6000L
RollingPts = 49      ; number of points to sample max and average on ishort
;   nominal is HK comes around every 49
;
; defaults below will be overwritten with data from SASP file
;
pressure = 819E2  ; default for Boulder; overwrite
year = 2016L

CASE UnitNumber OF
  0: BEGIN
    ConstOffsetA = 40.  ; take out guess at zero offset, makes plots a little cleaner, tiny effect on Rayleigh calculations
    ConstOffsetB = 40.
    ConstOffsetC = 40.
    ConstOffsetD = 40.
    ; from night or covered data
  END
  1: BEGIN
    ConstOffsetA = 40.  ; take out guess at zero offset, makes plots a little cleaner, tiny effect on Rayleigh calculations
    ConstOffsetB = 40.
    ConstOffsetC = 40.
    ConstOffsetD = 40.
    rotationrate = 4675.34   ; ms per radian, see unit number statements below
    ; from night or covered data
  END
  2: BEGIN
    ConstOffsetA = 41.50  ; take out guess at zero offset, makes plots a little cleaner, tiny effect on Rayleigh calculations
    ConstOffsetB = 27.34
    ConstOffsetC = 40.71
    ConstOffsetD = 40.04
    rotationrate = 4675.34   ; ms per radian, see unit number statements below
  END
  3: BEGIN
    ConstOffsetA = 45.31
    ConstOffsetB = 28.53
    ConstOffsetC = 45.35
    ConstOffsetD = 45.35
    rotationrate = 4675.34   ; ms per radian, see unit number statements below
  END
  ELSE: BEGIN
    ConstOffsetA = 40.
    ConstOffsetB = 40.
    ConstOffsetC = 40.
    ConstOffsetD = 40.
  END
ENDCASE
rotrate = 1000./(rotationrate*!DTOR)

CASE filetype OF
   1: BEGIN
     numcommas = 12
   END
ENDCASE


PhotoA = LONARR(nlong)   ; long gate
PhotoB = PhotoA
PhotoC = PhotoA
PhotoD = PhotoA
PhotoAsh = PhotoA        ; short gate
PhotoBsh = PhotoA
PhotoCsh = PhotoA
PhotoDsh = PhotoA
;PhotoLong = LONARR(nlong, 4)
;PhotoSh = PhotoLong
Seconds = DBLARR(nlong)

homep = FLTARR(nlong)
MicroUsed = homep
Azimuth = homep

Te = FLTARR(nshort)
Lat = Te
Lon = Te
GPSHr = Te
GPSReadSeconds = Te
CorrElev = Te
MonthDay = Te
HKSeconds = DBLARR(nshort)
Yaw = Te
Pitch = Te
Roll = Te
BaromPr = Te
BaromTe = Te
ModeFlag = LONARR(nshort)
GateLgArr = Te
GateShArr = Te
PhotoOffArr = Te
GPSTimeIndices = LONARR(nshort)


RollingArray = REPLICATE(RollingPts, 4)

int1 = 0L
int2 = 0L
int3 = 0L
int4 = 0L
int5 = 0L
int7 = 0L
int8 = 0L
int9 = 0L
ii = 0L
;
; read file
;
int10 = 0L
caseflag = 0
float1 = 0.0
float2 = 0.0
float3 = 0.0
ione = 0L
itwo = 0L
ithree = 0L
ifour = 0L
ifive = 0L
badstringcount = 0L

;if !THISMACHINE EQ 'murphyg5unix' THEN inpath = '/Volumes/Macintosh HD/Data/
fname = inpath + filename + suffix
tempstring = 'string
recentUDTHr = !VALUES.F_NAN
recentGPSSeconds = !VALUES.F_NAN
;
; loop to read SASP file
;
OPENR, InUnit, fname, /GET_LUN
IF skipstart GT 0 THEN FOR i = 0, skipstart-1 DO READF, InUnit, tempstring  ; first line(s) often garbage
WHILE NOT EOF(InUnit) DO BEGIN
  if ii mod 100000L EQ 0L then print, ii
  READF, InUnit, tempstring
  bytestring = BYTE(tempstring)
  IF MAX(bytestring) GT 57 or MIN(bytestring) LT 10 THEN badstring = 1 ELSE badstring = 0
  IF bytestring[0] EQ 36 then badstring = 0  ; $ starts vectornav lines
  IF badstring EQ 0 THEN BEGIN
    commas = where(bytestring[0:N_ELEMENTS(bytestring)-2] EQ 44B, commacount)
    IF commacount NE numcommas then badstring = 1
  ENDIF
  IF badstring EQ 0 THEN BEGIN
    READS, tempstring, int1, int2, int3, int4, int5, int6, int7, int8, int9, caseflag, float1, float2, float3
    PhotoAsh[ii] = int1
    PhotoBsh[ii] = int2
    PhotoCsh[ii] = int3
    PhotoDsh[ii] = int4
    PhotoA[ii] = int5
    PhotoB[ii] = int6
    PhotoC[ii] = int7
    PhotoD[ii] = int8
    Seconds[ii] = DOUBLE(int9)
    ;UDT[ii] = 1.0D*recentUDTHr + (int9*(millisscale/1000.) - recentGPSSeconds)/3600.0D

    IF caseflag EQ 0 THEN BEGIN
      azimuth[ii] = float1
      homep[ii] = float2
      MicroUsed[ii] = float3
    ENDIF ELSE BEGIN
      indx = (ii-1) > 0
      azimuth[ii] = !VALUES.F_NAN
      homep[ii] = !VALUES.F_NAN
      MicroUsed[ii] = MicroUsed[indx]
      IF caseflag EQ 1 THEN BEGIN
        lat[ione] = float1
        lon[ione] = float2
        Te[ione] = float3
        ione +=1
      ENDIF
      IF caseflag EQ 2 THEN BEGIN
        GPSHr[itwo] = float1
        MonthDay[itwo] = float2
        GPSReadSeconds[itwo] = float3/100.
        HKSeconds[itwo] = Seconds[ii]
        GPSTimeIndices[itwo] = ii
        itwo +=1
      ENDIF
      IF caseflag EQ 3 THEN BEGIN
        Yaw[ithree] = float1
        Pitch[ithree] = float2
        Roll[ithree] = float3
        ithree +=1
      ENDIF
      IF caseflag EQ 4 THEN BEGIN
        BaromPr[ifour] = float1
        BaromTe[ifour] = float2
        Modeflag[ifour] = LONG(float3 + 0.5)
        ifour +=1
      ENDIF
      IF caseflag EQ 5 THEN BEGIN
        GateLgArr[ifive] = float1
        GateShArr[ifive] = float2
        PhotoOffArr[ifive] = float3
        ifive +=1
      ENDIF
    ENDELSE
    ii +=1
  ENDIF ELSE BEGIN
    if bytestring[0] NE 36 THEN BEGIN
      ;print, 'bad string', ii, ' ', tempstring
      badstringcount += 1L
    ENDIF
  ENDELSE
ENDWHILE
FREE_LUN, InUnit
print, badstringcount, ' bad strings'

ii -= 1  ; eliminate last line because it might be incomplete
usethese1 = LINDGEN(ii)
photoA = photoA[usethese1]
photoB = photoB[usethese1]
photoC = photoC[usethese1]
photoD = photoD[usethese1]
maxposs = 2L^20
; Arduino negatives (unsigned) become very large positive 
;  2^20 is convenient rather than an exact threshold
;  recover using deliberate overflow
WasNeg = WHERE(photoA GT (maxposs),NHi)
IF NHi GT 0 THEN photoA[WasNeg] = (photoA[WasNeg]*16L)/16L
WasNeg = WHERE(photoB GT (maxposs),NHi)
IF NHi GT 0 THEN photoB[WasNeg] = (photoB[WasNeg]*16L)/16L
WasNeg = WHERE(photoC GT (maxposs),NHi)
IF NHi GT 0 THEN photoC[WasNeg] = (photoC[WasNeg]*16L)/16L
WasNeg = WHERE(photoD GT (maxposs),NHi)
IF NHi GT 0 THEN photoD[WasNeg] = (photoD[WasNeg]*16L)/16L

usethese = WHERE((photoA GE 0) AND (photoB GE 0) and (photoC GE 0) and (photoD GE 0) $
  and (photoA LT maxposs) AND (photoB LT maxposs) and (photoC LT maxposs) and (photoD LT maxposs)  $
  and (photoAsh LT maxposs) AND (photoBsh LT maxposs) and (photoCsh LT maxposs) and (photoDsh LT maxposs) $
  and ((photoA+photoB+photoC+photoD) NE 0), npoints)
; eliminate points where gate overran length because of reading GPS
; last because Arduiino writes all zeros if computation time overruns the integration time

photoA = photoA[usethese]
photoB = photoB[usethese]
photoC = photoC[usethese]
photoD = photoD[usethese]
photoAsh = photoAsh[usethese]
photoBsh = photoBsh[usethese]
photoCsh = photoCsh[usethese]
photoDsh = photoDsh[usethese]
homep = homep[usethese]
azimuth = azimuth[usethese]
Seconds = Seconds[usethese]*(millisscale/1000.)
MicroUsed = MicroUsed[usethese]*100.    ; factor 100 is because SASP Arduino code writes this in tenths of ms
IF FixAzimuthFlag GT 0 THEN BEGIN
    IF STRUPCASE(filename) EQ 'R1510141000' THEN BEGIN
      ; noisy homep so Arduino kept going over threshold, but good enough to sync to
      azimuth = Lindgen(N_elements(homep)) + 870L
      azimuth -= 2l*(azimuth GT 50000L)
      azimuth -= 3l*(azimuth GT 60000L)
      azimuth -= 2l*(azimuth GT 70000L)
      azimuth -= 4l*(azimuth GT 80000L)
      azimuth -= 3l*(azimuth GT 90000L)
      azimuth -= 3l*(azimuth GT 100000L)
      azimuth -= 3l*(azimuth GT 110000L)
      azimuth = (360.0/978.0)*(azimuth MOD 978L)
      stop
    ENDIF
ENDIF
;
; housekeeping type variables
;   for simplicity truncate to shortest of the rotating files; may lose a fraction of a second
;
ishort = (((ione < itwo ) < ithree) < ifour) < ifive
ishort= (((ione < itwo ) < ithree)) < ifive
te = te[0:ishort-1]
lat = lat[0:ishort-1]
lon = lon[0:ishort-1]
;CorrElev = CorrElev[0:ishort-1]

GPSHr = GPSHr[0:ishort-1]
MonthDay = MonthDay[0:ishort-1]
IF MonthDay[ishort-1] GT 1300L THEN BEGIN
  Year = 2000L + FLOOR(MonthDay[ishort-1]/10000L)
  MonthDay = MonthDay MOD 10000L
ENDIF
GPSReadSeconds = GPSReadSeconds[0:ishort-1]
HKSeconds = HKSeconds[0:ishort-1]*(millisscale/1000.)

Yaw = Yaw[0:ishort-1]
Pitch = Pitch[0:ishort-1]
Roll = Roll[0:ishort-1]
BaromPr = BaromPr[0:ishort-1]
BaromTe = BaromTe[0:ishort-1]
ModeFlag = FIX(ModeFlag[0:ishort-1])

vscale = 4.95
te = (te*(vscale/1023.)-.5)*100.  
;
; fill and process azimuth, UDT
;
missaz = WHERE(FINITE(azimuth) EQ 0, nmiss)   ; azimuth has missing values due to rotating housekeeping variables
IF nmiss GT 0 THEN BEGIN
  previndx = (missaz - 1) > 0L
  azimuth[missaz] = azimuth[previndx] + (Seconds[missaz] - Seconds[previndx])*rotrate
  ; only fixes single missing values
ENDIF
; fix no udt before first one in slower data (NOT USED WITH CODE BELOW)
;aa = FINITE(udt[0:(2000 < (ii-1))])
;HaveUDT = WHERE(aa EQ 1)
;IF HaveUDT[0] GT 0 THEN udt[0:HaveUDT[0]-1] = udt[HaveUDT[0]] + (seconds[0:HaveUDT[0]-1] - seconds[HaveUDT[0]])/3600.å
;
; --------------------------- put times from internal clock onto GPS times -------------------------------------------
;   uses a linear fit to account for clock drift, could also use interpolation or other methods
;   also assume that any lat and lon before GPS acquires are same as first GPS point
;
HadGPS = WHERE(GPSReadSeconds GT 0 AND MonthDay GT 0, NHadGPS, COMPLEMENT = NoGPS, NCOMPLEMENT = nNoGPS)
IF HadGPS[0] GT 0 THEN BEGIN
  MonthDay[0:HadGPS[0]-1] = MonthDay[HadGPS[0]]
  ; may fail if go through midnight during period before GPS capture
  lat[0:HadGPS[0]-1] = lat[HadGPS[0]]
  lon[0:HadGPS[0]-1] = lon[HadGPS[0]]
ENDIF
Month = FLOOR(MonthDay/100)
Day = MonthDay - Month*100
IF nHadGPS GT 2 THEN BEGIN
  IF MonthDay[ishort-1] GT MonthDay[HadGPS[0]] THEN BEGIN
    Month = FLOOR(MonthDay/100)
    TempDays = JULDAY(Month, MonthDay-Month, Year, 1, 0, 0)
    GPSHr = GPSHr + 24.0D*(TempDays - TempDays[0])
  ENDIF
  timefit = LINFIT(GPSHr[HadGPS], GPSReadSeconds[HadGPS], /DOUBLE)
  print, timefit, ' fit of internal times to GPS times'
ENDIF ELSE BEGIN
  PRINT, 'Not enough GPS readings to compute times'
  STOP
ENDELSE
UDTofHK = (HKSeconds - timefit[0])/timefit[1] ; tiny bit off for other HK parameters
UDT = (Seconds - timefit[0])/timefit[1]
docheckplot = 0
IF docheckplot THEN BEGIN
  aa = 33500.+LINDGEN(1000)  ; plot a bit from middle
  plot_io, photoash[aa] - 22., color=3
  oplot, photobsh[aa] - 22., color=4
  oplot, (photocsh[aa] - 22.)/1.5, color=2
  oplot, (photodsh[aa] - 22.)/2., color=0
ENDIF

CASE GateType OF
  0: BEGIN  ; 20140703
    shortgate = GateShArr[0]*1E-6
    longgate =  GateLgArr[0]*1E-6
    shorttimeoffset = -(longgate + 0.5*shortgate)
    longtimeoffset = -0.5*longgate
    ;    sunthreshold = 2000.  ; for finding sun in short gate
    ;    UseForThreshold = 'A'
    ; sunthreshold = 100.  ; for finding sun in short gate
    ; UseForThreshold = 'C'
  END
ENDCASE
;
; -------------------------------------- BUILD MERGED DATA ----------------------------------------
;
; technical point: using short or long gate means a tiny change in time
;   time (UDT) is computed as a single time for each measurement, ignoring gate length
;   azimuths include the small change from gate length. This is why there are 4 of them: one color might be on short gate, another on long
;   gate length difference is about 0.3 degrees
;
gateratio = longgate/shortgate
;strratio = STRTRIM(STRING(gateratio), 2)
PhotoAMerge = FLOAT(PhotoA)
PhotoBMerge = FLOAT(PhotoB)
PhotoCMerge = FLOAT(PhotoC)
PhotoDMerge = FLOAT(PhotoD)
AzimuthA = FLOAT(Azimuth)
AzimuthB = AzimuthA
AzimuthC = AzimuthA
AzimuthD = AzimuthA

UDT += longtimeoffset/3600.
;
; find and subtract small digitizer offset between short and long gates
;   fitted slope calculation
;   -------  20170913 I think mean offset below is simpler and better --------
;
;HiSigA = WHERE(PhotoA GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
;IF nlosig GT 1E6 THEN BEGIN ; fits just get really slow if too many points, so decimate
;  LoSig = LoSig[10L*LINDGEN(nlosig/10L)]
;ENDIF
;IF nlosig GT 0 THEN BEGIN
;  HiLoFit = LADFIT(PhotoA[LoSig], PhotoASh[LoSig])  ; less bias to put noisy short as y!
;  print, 'A', HiLoFit[0], 1.0/hilofit[1], GateRatio
;  AOffset = HiLoFit[0]
;  IF nhisig GT 0 THEN BEGIN
;    PhotoAMerge[HiSigA] = (PhotoAsh[HiSigA]-HiLoFit[0])*gateratio
;    AzimuthA[HiSigA] += rotrate*(shorttimeoffset - longtimeoffset)
;    ;IF STRUPCASE(UseForThreshold) EQ 'A' THEN UDT[HiSigA] += (shorttimeoffset - longtimeoffset)/3600.
;  ENDIF
;ENDIF ELSE PhotoAMerge = PhotoAsh
;HiSigB = WHERE(PhotoB GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
;IF nlosig GT 1E6 THEN BEGIN ; fits just get really slow
;  LoSig = LoSig[10L*LINDGEN(nlosig/10L)]
;ENDIF
;IF nlosig GT 0 THEN BEGIN
;  HiLoFit = LADFIT(PhotoB[LoSig], PhotoBSh[LoSig]) 
;  print, 'B', HiLoFit[0], 1.0/hilofit[1], GateRatio
;  BOffset = HiLoFit[0]
;  IF nhisig GT 0 THEN BEGIN
;    PhotoBMerge[HiSigB] = (PhotoBsh[HiSigB]-HiLoFit[0])*gateratio
;    AzimuthB[HiSigB] += rotrate*(shorttimeoffset - longtimeoffset)
;  ENDIF
;ENDIF ELSE PhotoBMerge = PhotoBsh
;HiSigC = WHERE(PhotoC GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
;IF nlosig GT 1E6 THEN BEGIN ; fits just get really slow
;  LoSig = LoSig[10L*LINDGEN(nlosig/10L)]
;ENDIF
;IF nlosig GT 0 THEN BEGIN
;  HiLoFit = LADFIT(PhotoC[LoSig], PhotoCSh[LoSig])  
;  print, 'C', HiLoFit[0], 1.0/hilofit[1], GateRatio
;  COffset = HiLoFit[0]
;  IF nhisig GT 0 THEN BEGIN
;    PhotoCMerge[HiSigC] = (PhotoCsh[HiSigC]-HiLoFit[0])*gateratio
;    AzimuthC[HiSigC] += rotrate*(shorttimeoffset - longtimeoffset)
;  ENDIF
;ENDIF ELSE PhotoCMerge = PhotoCsh
;HiSigD = WHERE(PhotoD GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
;IF nlosig GT 1E6 THEN BEGIN ; fits just get really slow
;  LoSig = LoSig[10L*LINDGEN(nlosig/10L)]
;ENDIF
;IF nlosig GT 0 THEN BEGIN
;  HiLoFit = LADFIT(PhotoD[LoSig], PhotoDSh[LoSig])  
;  print, 'D', HiLoFit[0], 1.0/hilofit[1], GateRatio
;  DOffset = HiLoFit[0]
;  IF nhisig GT 0 THEN BEGIN
;    PhotoDMerge[HiSigD] = (PhotoDsh[HiSigD]-HiLoFit[0])*gateratio
;    AzimuthD[HiSigD] += rotrate*(shorttimeoffset - longtimeoffset)
;  ENDIF
;ENDIF ELSE PhotoDMerge = PhotoDsh
;
;
; find and subtract small digitizer offset between short and long gates
;   fixed slope calculation
;
HiSigA = WHERE(PhotoA GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
IF nlosig GT 0  THEN BEGIN
  AOffset = MEAN(PhotoASh[LoSig] - PhotoA[LoSig]/gateratio)
  print, 'A ', AOffset
  IF nhisig GT 0 THEN BEGIN
    PhotoAMerge[HiSigA] = (PhotoAsh[HiSigA]-AOffset)*gateratio
    AzimuthA[HiSigA] += rotrate*(shorttimeoffset - longtimeoffset)
   ENDIF
ENDIF ELSE PhotoAMerge = PhotoAsh
HiSigB = WHERE(PhotoB GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
IF nlosig GT 0 THEN BEGIN
  BOffset = MEAN(PhotoBSh[LoSig] - PhotoB[LoSig]/gateratio)
  print, 'B ', BOffset
  IF nhisig GT 0 THEN BEGIN
    PhotoBMerge[HiSigB] = (PhotoBsh[HiSigB]-BOffset)*gateratio
    AzimuthB[HiSigB] += rotrate*(shorttimeoffset - longtimeoffset)
  ENDIF
ENDIF ELSE PhotoBMerge = PhotoBsh
HiSigC = WHERE(PhotoC GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
IF nlosig GT 0 THEN BEGIN
  COffset = MEAN(PhotoCSh[LoSig] - PhotoC[LoSig]/gateratio)
  print, 'C ', COffset
  IF nhisig GT 0 THEN BEGIN
    PhotoCMerge[HiSigC] = (PhotoCsh[HiSigC]-COffset)*gateratio
    AzimuthC[HiSigC] += rotrate*(shorttimeoffset - longtimeoffset)
  ENDIF
ENDIF ELSE PhotoCMerge = PhotoCsh
HiSigD = WHERE(PhotoD GT HiThres, nhisig,COMPLEMENT = LoSig, NCOMPLEMENT = nlosig)
IF nlosig GT 0 THEN BEGIN
  DOffset = MEAN(PhotoDSh[LoSig] - PhotoD[LoSig]/gateratio)
  print, 'D ', DOffset
  IF nhisig GT 0 THEN BEGIN
    PhotoDMerge[HiSigD] = (PhotoDsh[HiSigD]-DOffset)*gateratio
    AzimuthD[HiSigD] += rotrate*(shorttimeoffset - longtimeoffset)
  ENDIF
ENDIF ELSE PhotoDMerge = PhotoDsh


PhotoAMerge -= ConstOffsetA
PhotoBMerge -= ConstOffsetB
PhotoCMerge -= ConstOffsetC
PhotoDMerge -= ConstOffsetD
;
; ----------------------------- Short version of data on each HK time (about 1.5 s instead of 33 ms)
;
HKPhotoMax = FLTARR(ishort, 4)
HKPhotoAvg = HKPhotoMax
FOR i = 0, ishort-1 DO BEGIN
  Theseindices = ((LINDGEN(RollingPts) + GPSTimeIndices[i]) > 0L) < (npoints - 1L)
  HKPhotoMax[i, 0] = MAX(PhotoAMerge[TheseIndices])
  HKPhotoMax[i, 1] = MAX(PhotoBMerge[TheseIndices])
  HKPhotoMax[i, 2] = MAX(PhotoCMerge[TheseIndices])
  HKPhotoMax[i, 3] = MAX(PhotoDMerge[TheseIndices])
  HKPhotoAvg[i, 0] = TOTAL(PhotoAMerge[TheseIndices])/FLOAT(RollingPts)
  HKPhotoAvg[i, 1] = TOTAL(PhotoBMerge[TheseIndices])/FLOAT(RollingPts)
  HKPhotoAvg[i, 2] = TOTAL(PhotoCMerge[TheseIndices])/FLOAT(RollingPts)
  HKPhotoAvg[i, 3] = TOTAL(PhotoDMerge[TheseIndices])/FLOAT(RollingPts)
ENDFOR
;
; -------------------------------------- CALCULATED PARAMETERS -----------------------------------------
;
; solar zenith
;
UDtForZenith = [udt[0L], UDTofHK, UDT[npoints-1L]]
; use housekeeping spacing for fewer calculations, then interpolate
latforzenith = [lat[0L], lat, lat[ishort-1L]]
lonforzenith = [lon[0L], lon, lon[ishort-1L]]
SolarZenithsh = latforzenith ; size
SolarAzimuthsh = latforzenith
FOR i = 0L, N_ELEMENTS(latforzenith)-1L DO BEGIN
  sunpos = sunPositionsingle(latforzenith[i], lonforzenith[i], year, month[0], day[0], UDtForZenith[i]*3600.)
  SolarZenithsh[i] = 90.0 - sunpos[0L]
  SolarAzimuthsh[i] = sunpos[1L]
ENDFOR
SolarZenith = INTERPOL(SolarZenithsh, UDtForZenith, UDT)
SolarAzimuth = INTERPOL(SolarAzimuthsh, UDtForZenith, UDT)
;YawforInterp = [Yaw[0L], Yaw, Yaw[ishort-1L]]
;YawX = SIN(YawforInterp*!DTOR)
;YawY = COS(YawforInterp*!DTOR)
;YawXEvery = INTERPOL(YawX, UDtForZenith, UDT)
;YawYEvery = INTERPOL(YawY, UDtForZenith, UDT)
;YawEvery = ATAN(YawXEvery,YawYEvery)/!DTOR
;
; ---------------------------- write output files -----------------------------------------
;
IF WriteFileFlag EQ 1 THEN BEGIN
  ;
  ; using IGOR text file format here, a simple text format that holds multiple variables and of course easy to read into IGOR
  ;
  Aformat = '(F14.7, 4F12.2, 4F10.3, F8.1)'
  nfilesplit = 1  ; for future expansion if files get too big
  FOR ifile = 0, nfilesplit-1 DO BEGIN
    namesuf = STRTRIM(FIX(MonthDay[0]),2) + WaveNameSuffix 
    ;
    ; time series
    ;
    OPENW, OutFile, inpath + filename + 'A'+'.itx', /GET_LUN, MACTYPE = 'IGTX'
    PRINTF, OutFile, 'IGOR'
    Outstring = 'WAVES /N=('+STRTRIM(npoints,2)+') /O '
    Outstring = Outstring+'UDT'+namesuf+', phA'+namesuf+', phB'+namesuf+', phC'+namesuf+', phD'+namesuf
    Outstring = Outstring+', AzA'+namesuf+', AzB'+namesuf+', AzC'+namesuf+', AzD'+namesuf+', homep'+namesuf
    PRINTF, OutFile, Outstring
    PRINTF, OutFile, 'BEGIN'
    FOR iout = 0L,npoints-1 DO BEGIN
      PRINTF, OutFile, UDT[iout], photoAMerge[iout], photoBMerge[iout], photoCMerge[iout], photoDMerge[iout], $
        AzimuthA[iout], AzimuthB[iout], AzimuthC[iout], AzimuthD[iout], homep[iout], FORMAT = AFormat
    ENDFOR
    PRINTF, OutFile, 'END'
    CLOSE, OutFile
    ;
    ; housekeeping and things on slower time base
    ;
    OPENW, OutFile, inpath + filename + 'H'+'.itx'
    PRINTF, OutFile, 'IGOR'
    Outstring = 'WAVES /N=('+STRTRIM(ishort,2)+') /O '
    Outstring = Outstring+'HKTime'+namesuf+', te'+namesuf+', pitch'+namesuf+', roll'+namesuf+', yaw'+namesuf
    Outstring = Outstring+', lat'+namesuf+', lon'+namesuf
    Outstring = Outstring+', BaromPr'+namesuf+', BaromTe'+namesuf+', SolarZenith'+namesuf+', SolarAzimuth'+namesuf
    Outstring = Outstring+', ModeFlag'+namesuf
    Outstring = Outstring+', HKAAvg'+namesuf+', HKBAvg'+namesuf+', HKCAvg'+namesuf+', HKDAvg'
    Outstring = Outstring+', HKAMax'+namesuf+', HKBMax'+namesuf+', HKCMax'+namesuf+', HKDMax'
    PRINTF, OutFile, Outstring
    PRINTF, OutFile, 'BEGIN'
    FOR iout = 0L, ishort-1 DO BEGIN
      PRINTF, OutFile, UDTofHK[iout], te[iout], pitch[iout], roll[iout], yaw[iout], $
      lat[iout], lon[iout], BaromPr[iout], $
      baromte[iout], SolarZenithsh[iout], SolarAzimuthsh[iout], ModeFlag[iout], $
      HKPhotoAvg[iout, 0], HKPhotoAvg[iout, 1], HKPhotoAvg[iout, 2], HKPhotoAvg[iout, 3], $
      HKPhotoMax[iout, 0], HKPhotoMax[iout, 1], HKPhotoMax[iout, 2], HKPhotoMax[iout, 3]
    ENDFOR
    PRINTF, OutFile, 'END'
    PRINTF, OutFile, 'WAVES /N=(16) /O params'+namesuf
    PRINTF, OutFile, 'BEGIN'
    PRINTF, OutFile, UnitNumber
    PRINTF, OutFile, shortgate
    PRINTF, OutFile, longgate
    PRINTF, OutFile, rotrate
    PRINTF, OutFile, timefit[0]
    PRINTF, OutFile, timefit[1]
    PRINTF, OutFile, AOffset
    PRINTF, OutFile, BOffset
    PRINTF, OutFile, COffset
    PRINTF, OutFile, DOffset
    PRINTF, OutFile, ConstOffsetA
    PRINTF, OutFile, ConstOffsetB
    PRINTF, OutFile, ConstOffsetC
    PRINTF, OutFile, ConstOffsetD
    PRINTF, OutFile, Year
    PRINTF, OutFile, RollingPts  ; change N= above if more parameters added
    PRINTF, OutFile, 'END'
    
    FREE_LUN, OutFile
  ENDFOR
ENDIF

RETURN
END