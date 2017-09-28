FUNCTION sunPositionsingle, lat, lon, year, month, day, UDT

; For logic and names, see Spencer, J.W. 1989. Solar Energy. 42(4):353
; adapted from modified R code on web with fixes to mistakes in Michalsky
twopi = 2.0*!PI

;   deg2rad = !DTOR

; Get day of the year, e.g. Feb 1 = 32, Mar 1 = 61 on leap years
cummonthdays =     [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334]
cummonthdaysleap = [0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335]
if (Year MOD 4) EQ 0 THEN $ 
  dayyear = day + cummonthdaysleap[month-1] - 1 ELSE $
  dayyear = day + cummonthdays[month-1] - 1
hour = UDT/3600. ; hour + min/60. + sec/3600.
; The input to the Atronomer's almanach is the difference between
; the Julian date and (noon, 1 January 2000)
; modified to delta from 2013 for better single precision
deltayr2 = year - 2013L 
nleap2 = FLOOR(deltayr2/4. + 0.01)
time2 =  deltayr2 * 365.0 + nleap2 + dayyear + hour / 24.0 - 0.5 
; 0.5 is because julday is at noon and formulas below are julday

; Ecliptic coordinates
; Mean longitude
;mnlong = 280.460D + 0.9856474D*time ; .463 best eclong but .459 best ra
mnlong = 281.29860 + 0.985647*time2   
mnlong = mnlong MOD 360.0
;mnlong += 360.0*(mnlong LT 0.0)
; Mean anomaly
mnanom = 358.14382 + 0.9856 * time2
mnanom = mnanom MOD 360.0
;mnanom += 360.0*(mnanom LT 0)
mnanom = mnanom *!DTOR
; Ecliptic longitude and obliquity of ecliptic
;eclong = mnlong + 1.915 * sin(mnanom) + 0.020 * sin(2 * mnanom)
eclong = mnlong + 1.915 * sin(mnanom) + 0.0205 * sin(2 * mnanom)
;print, mnanom
eclong = eclong MOD 360.0
eclong += 360.0*(eclong LT 0)
eclong = eclong * !DTOR

; Celestial coordinates
; Right ascension and declination
;num =  0.91751185D* sin(eclong)
num =  0.91751185* sin(eclong)
den = cos(eclong)
ra = atan(num / den)
ra += !PI*(den LT 0.0)
ra += 2*!PI*(den GE 0.0 AND num LT 0.0)
dec = asin(0.39770844* sin(eclong))      

; Local coordinates
; Greenwich mean sidereal time
gmst = 6.75333 + 0.0657098242*time2 + hour
gmst = gmst MOD 24.0
gmst += 24.0*(gmst LT 0.0)
; Local mean sidereal time
lmst = gmst + lon/15.
lmst = lmst MOD 24.0
lmst += 24.0*(lmst LT 0.0)

; Hour angle
ha = lmst * 15.0* !DTOR - ra
ha += twopi*(ha LT -!PI)
ha -= twopi*(ha GT !PI)

;Azimuth and elevation
sinlat = sin(lat*!DTOR)
el = asin(sin(dec) * sinlat + cos(dec) * cos(lat*!DTOR) * cos(ha))
az = asin(-cos(dec) * sin(ha)/cos(el))

cosAzPos = 0.0 LE (sin(dec) - sin(el) * sinlat)
sinAzNeg = sin(az) LT 0.0
az+= twopi*(cosAzPos*sinAzNeg)
notcosAzPos = ABS(1.0-cosAzPos)
az = az + (!DPI - 2.0*az)*notcosAzPos
;az[!cosAzPos] <- pi - az[!cosAzPos]

el = el /!DTOR
az = az /!DTOR

oblqec = 23.435* !DTOR
;print, time2, ra/!DTOR, dec/!DTOR, eclong/!DTOR, ha/!DTOR, lmst, gmst, el
return, [el, az, ra/!DTOR, dec/!DTOR, eclong/!DTOR, oblqec/!DTOR]
;return, [el, az, ra/!DTOR, dec/!DTOR]
END