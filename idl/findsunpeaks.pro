PRO FindSunPeaks, SunPhoto, UDT, SunThreshold, $
  npeaks, MidUDT, LoIndices, HiIndices, MidYaw, PeakTypeFlags, $
  ROTRATE = RotRate, TIMESPACING = TimeSpacing, $
  MAXWIDTHANGLE = MaxWidthAngle, CLOSESTABOVE = ClosestAbove, GAPANGLE = GapAngle, $
  MINSUNTHRESHOLD = MinSunThreshold

IF N_ELEMENTS(ROTRATE) LT 1 THEN RotRate = 360./30.        ; degrees
IF N_ELEMENTS(TIMESPACING) LT 1 THEN TimeSpacing = 0.030   ; seconds
IF N_ELEMENTS(MAXWIDTHANGLE) LT 1 THEN MaxWidthAngle = 9.  ; degrees
IF N_ELEMENTS(CLOSESTABOVE) LT 1 THEN ClosestAbove = 30.   ; degrees
IF N_ELEMENTS(GAPANGLE) LT 1 THEN GapAngle = 600.          ; degrees
IF N_ELEMENTS(MINSUNTHRESHOLD) LT 1 THEN MinSunThreshold = 0.01*SunThreshold
; probably code problems if this is less than 360., certainly if less than 180
nSmooth = 7  ; used for finding peaks below SunThreshold
;smallpkratio = 0.01 ; small peaks must be larger than smallpkratio*sunthreshold

nToSide = CEIL(MaxWidthAngle/(rotrate*TimeSpacing))
TimeToSide = (MaxWidthAngle/rotrate)/3600.


AboveThreshold = SunPhoto GT sunthreshold
npts = N_ELEMENTS(SunPhoto)
ThresDiff = AboveThreshold[1L:npts-1L] - AboveThreshold[0L:npts-2L]
ThresDiff = [0, ThresDiff]
RisingEdge = WHERE(ThresDiff EQ 1, nRisingEdge)
 
IF nRisingEdge GT 0 THEN BEGIN
  ; take out last or first sun encounter if too close to end of file
   IF RisingEdge[nRisingEdge-1] GT npts-(nToSide+2) THEN BEGIN
    RisingEdge = RisingEdge[0L:nRisingEdge-2L]
    nRisingEdge = nRisingEdge-1L
  ENDIF
  IF RisingEdge[0] LT (nToSide+2) THEN BEGIN
    RisingEdge = RisingEdge[1L:nRisingEdge-1L]
    nRisingEdge = nRisingEdge-1L
  ENDIF
ENDIF
nPeaks = nRisingEdge

IF nRisingEdge GT 0 THEN BEGIN
  LoIndices = LONARR(nRisingEdge)
  HiIndices = LoIndices
  MidSeconds = DBLARR(nRisingEdge)
  FOR ihigh = 0L, nRisingEdge-1L DO BEGIN
    LookForSatRegion = AboveThreshold[RisingEdge[ihigh] + LINDGEN(nToSide)]
    TimeDiff = UDT[LookForSatRegion]-UDT[LookForSatRegion[0]]
    ThisAbove = WHERE((LookForSatRegion EQ 1) AND (TimeDiff LE TimeToSide), nThisAbove)
    LoIndices[ihigh] = ThisAbove[0]
    HiIndices[ihigh] = ThisAbove[nThisAbove-1]
  ENDFOR
  LoIndices = LoIndices + RisingEdge
  HiIndices = HiIndices + RisingEdge
  localwidths = HiIndices - LoIndices
  MidSeconds = 1800.*(UDT[LoIndices] + UDT[HiIndices])
  midindices = ROUND((LoIndices + HiIndices)/2.)
  IF nRisingEdge GT 1 THEN BEGIN
    SunDeltaAngles = (MidSeconds[1:nRisingEdge-1] - MidSeconds[0:nRisingEdge-2])*rotrate
    ;
    ; code for too close 
    ;
    TooClose = WHERE(SunDeltaAngles LT ClosestAbove, nTooClose)
    IF nTooClose GT 0 THEN BEGIN
      Ones = REPLICATE(1, nRisingEdge)
      ; take wider one because noise on edge of peak will give a narrow side peak
      FOR iclose = 0, nTooClose-1 DO BEGIN
        thisindx = TooClose[iclose]
        IF localwidths[thisindx + 1] GT localwidths[thisindx] THEN $
          Ones[thisindx] = 0 ELSE Ones[ThisIndx + 1] = 0 
      ENDFOR
      KeepPeaks = WHERE(Ones)
      LoIndices = LoIndices[KeepPeaks]
      HiIndices = HiIndices[KeepPeaks]
      midindices = midindices[KeepPeaks]
      MidSeconds = MidSeconds[KeepPeaks]
      localwidths = localwidths[KeepPeaks]
      nPeaks = nRisingEdge - nTooClose
     ENDIF 
  ENDIF
ENDIF
IF nPeaks GT 0 THEN PeakTypeFlags = INTARR(nPeaks)
;
; code for peaks too far apart: find local maxima 
;  (or no peaks above threshold at all)
;

IF NRisingEdge EQ 0 THEN BEGIN
  StartTooFar = [0L]
  EndTooFar = [npts-1]
  NTooFar = 1L
  Midseconds = [0.0]
  LoIndices = [0]
  HiIndices = [1]
ENDIF ELSE BEGIN
  IF nRisingEdge GT 2 THEN BEGIN
    ; gaps between peaks
    SunDeltaAngles = (MidSeconds[1:nPeaks-1] - MidSeconds[0:nPeaks-2])*rotrate
    TooFar = WHERE(SunDeltaAngles GT GapAngle, nTooFar)
    IF nTooFar GT 0 THEN BEGIN
      StartTooFar = Hiindices[TooFar]
      EndTooFar =   Loindices[TooFar+1]
    ENDIF
  ENDIF ELSE nToofar = 0
  IF (MidSeconds[0] - UDT[0])*rotrate GT GapAngle THEN BEGIN
    ; gap at beginning of file
    IF nTooFar GT 0 THEN BEGIN
      StartTooFar = [0L, StartTooFar]
      EndTooFar = [midIndices[0], EndTooFar]
    ENDIF ELSE BEGIN
      StartTooFar = [0L]
      EndTooFar = [midIndices[0]]
    ENDELSE
    nTooFar += 1
  ENDIF
  IF (UDT[npts-1L]*3600. - MidSeconds[nPeaks-1])*rotrate GT GapAngle THEN BEGIN
    IF nTooFar GT 0 THEN BEGIN
      StartTooFar = [StartTooFar, midindices[nPeaks-1]]
      EndTooFar = [EndTooFar, npts-1]
    ENDIF ELSE BEGIN
      StartTooFar = [midindices[nPeaks-1]]
      EndTooFar = [npts-1L]
    ENDELSE
    nTooFar += 1
  ENDIF
ENDELSE

iGapMax = 0L
IF nTooFar GT 0 THEN BEGIN
  SunPhotosmooth = smooth(SunPhoto, nSmooth)
  n180 = CEIL(180./(rotrate*TimeSpacing))
  n360 = ROUND(360./(rotrate*TimeSpacing))
  GapMaxs = LONARR(CEIL(npts/n360))  ; maximum...
  StartTooFar = StartTooFar + n180
  EndTooFar = EndTooFar - n180
  FOR igap = 0, nTooFar-1 DO BEGIN
    nrotations = CEIL((EndTooFar[igap] - StartTooFar[igap])/n360)
    FOR irot = 0L, nrotations-1L DO BEGIN
      begindx = StartTooFar[igap] + irot*n360
      endindx = (begindx + n360) < EndTooFar[igap]
      thisrotationsignal = SunPhotosmooth[begindx:endindx]
      localmax = MAX(thisrotationsignal, localmaxindx)
      IF localmax GT MinSunThreshold THEN BEGIN
        maxindx = localmaxindx + begindx
        GapMaxs[iGapMax] = maxindx
        iGapMax += 1
      ENDIF
    ENDFOR
  ENDFOR
  IF iGapMax GT 0 THEN GapMaxs = GapMaxs[0:iGapMax - 1]
ENDIF

IF iGapMax GT 0L THEN BEGIN
  IF nPeaks GT 0 THEN BEGIN
    LoIndices = [LoIndices, GapMaxs - nSmooth/2]
    HiIndices = [HiIndices, GapMaxs + nSmooth/2]
    MidSeconds = [MidSeconds, UDT[GapMaxs]*3600.]
    PeakTypeFlags = [PeakTypeFlags, REPLICATE(1, iGapMax)]
    nPeaks += iGapMax
    sortn = SORT(MidSeconds)
    LoIndices = LoIndices[sortn]
    HiIndices = HiIndices[sortn]
    MidSeconds = MidSeconds[sortn]
    PeakTypeFlags = PeakTypeFlags[sortn]
  ENDIF ELSE BEGIN
    LoIndices = GapMaxs - nSmooth/2
    HiIndices = GapMaxs + nSmooth/2
    MidSeconds = UDT[GapMaxs]
    PeakTypeFlags = REPLICATE(1, iGapMax)
  ENDELSE
ENDIF
MidUDT = MidSeconds/3600.

RETURN
END