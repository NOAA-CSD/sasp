function [rotationrate, shortgate, longgate, npoints, nhk, fastvar, hkvar] = readradiometerfile(fname, skipstart)
%Read program for SASP file
%    April 2016 split out reading file from rest of code
%   known issues: 
%     - will fail if there are no data points within reasonable range (positive but less than 2^20)
%     - may fail if file crosses into new year, not debugged for new month
%     - often valid GPS may not be present at start of file. Filling this in may fail across midnight
%       (don't turn on instrument a minute before midnight UDT)
%       (workaround is to skip start of file with skipstart variable)
%

%fname = '/Volumes/Data HD/Radiom/Tampa3/r0102041815.txt';   % put in input
fname = '/Volumes/Data HD/Radiom/Tampa3/temp.txt';   % put in input
skipstart = 10;


%  constants for reading and interpreting file
%
rotationrate = 5194.8;    % ms per radian, see unit number statements below
millisscale = 10.;        % factor in Arduino print statement
UnitNumber = 25;          % should put in inputs, put unit-specific information here
% UnitNumber for Cyprus unit is 20+number written on board in marker (e.g. 23 if three on board)
%
%
nlong = 800;    % maximum number of lines in a SASP file
% 9 hours is about a million points
nshort = nlong/20;
HiThres = 10000;      % when to transition from long to short gate (raw units)  % 6000L
RollingPts = 49;      % number of points to sample max and average on ishort
%   nominal is HK comes around every 49
%
% defaults below will be overwritten with data from SASP file
%
pressure = 819E2;
Year = 2016;

ConstOffsetA = 40.;  % take out guess at zero offset, makes plots a little cleaner, tiny effect on Rayleigh calculations
ConstOffsetB = 40.;
ConstOffsetC = 40.;
ConstOffsetD = 40.;

switch UnitNumber
    case 25 
    ConstOffsetA = 40.;  % here is where to put dark current measurements
    ConstOffsetB = 40.;
    ConstOffsetC = 40.;
    ConstOffsetD = 40.;
    % from night or covered data
    otherwise
end

%PhotoA = zeros(1, nlong);   % long gate
%PhotoAsh = PhotoA;        % short gate

Te = zeros(1, nshort);
Lat = Te;
Lon = Te;
GPSHr = Te;
GPSReadSeconds = Te;
MonthDay = Te;
HKSeconds = Te;
Yaw = Te;
Pitch = Te;
Roll = Te;
BaromPr = Te;
BaromTe = Te;
ModeFlag = Te;
GateLgArr = Te;
GateShArr = Te;
PhotoOffArr = Te;
GPSTimeIndices = Te;

RollingArray = zeros(1,4) + RollingPts;
%
% read file setupt
%
ione = 1;
itwo = 1;
ithree = 1;
ifour = 1;
ifive = 1;
badstringcount = 0;
recentUDTHr = NaN;
recentGPSSeconds = NaN;
%
% loop through SASP file
%
ii = 1;
InUnit = fopen(fname);
if skipstart > 0 
 for i = 1:skipstart
   tempstring = fgetl(InUnit);  % first line(s) sometimes garbage
   % or non-output such as barometer constants
 end
end
while ~feof(InUnit)
  if mod(ii,10000) == 0
    ii = ii;  % poor way of printing ii to command window
  end
  tempstring = fgetl(InUnit);
  badstring = 1;
  if tempstring(1) == '$'
    badstring = 0;   % starts commands recorded with output
  else
    sepstring = strsplit(tempstring, ',');
    numpieces = numel(sepstring);
    if numpieces == 13    % only use complete lines
      nums = str2num(tempstring);
      badstring = 0;
      PhotoAsh(ii) = nums(1);
      PhotoBsh(ii) = nums(2);
      PhotoCsh(ii) = nums(3);
      PhotoDsh(ii) = nums(4);
      PhotoA(ii)   = nums(5);
      PhotoB(ii)   = nums(6);
      PhotoC(ii)   = nums(7);
      PhotoD(ii)   = nums(8);
      Seconds(ii)  = nums(9);
      caseflag = nums(10);
      if caseflag == 0
        azimuth(ii) = nums(11);
        homep(ii) = nums(12);
        MicroUsed(ii) = nums(13);
      else
        azimuth(ii) = NaN;
        homep(ii) = NaN;
        indx = max (1, ii-1);
        MicroUsed(ii) = MicroUsed(indx);
        if caseflag == 1
          Lat(ione) = nums(11);
          Lon(ione) = nums(12);
          Te(ione)  = nums(13);
          ione = ione + 1;
        end
        if caseflag == 2
          GPSHr(itwo) = nums(11);
          MonthDay(itwo) = nums(12);
          GPSReadSeconds(itwo) = nums(13)/100;
          HKSeconds(itwo) = Seconds(ii);
          GPSTimeIndices(itwo) = ii;
          itwo = itwo + 1;
        end
        if caseflag == 3
          Yaw(ithree) = nums(11);
          Pitch(ithree) = nums(12);
          Roll(ithree) = nums(13);
          ithree = ithree + 1;
        end
        if caseflag == 4
          BaromPr(ifour) = nums(11);
          BaromTe(ifour) = nums(12);
          ModeFlag(ifour) = nums(13);
          ifour = ifour + 1;
        end
        if caseflag == 5
          GateLgArr(ifive) = nums(11);
          GateShArr(ifive) = nums(12);
          PhotoOffArr(ifive) = nums(13);
          ifive = ifive + 1;
        end
      end     
      ii = ii + 1;
    end
  end
  badstringcount = badstringcount + badstring;
end  % while eof
fclose(InUnit);
%
%   --------------- clean up possibly bad data ------------------
%
% Arduino negatives (unsigned) become very large positive 
%  2^20 is convenient rather than an exact threshold
%  recover using deliberate overflow
maxposs = 2^20;
WasNeg = find(PhotoA > maxposs);
if numel(WasNeg) > 0
  PhotoA(WasNeg) = 0;  % actually should be negative: see next line
%  PhotoA(WasNeg) = (PhotoA(WasNeg)*16L)/16L;  % don't know how to do this
%  in MATLAB
end
WasNeg = find(PhotoB > maxposs);
if numel(WasNeg) > 0
  PhotoB(WasNeg) = 0;  % actually should be negative: see next line
end
WasNeg = find(PhotoC > maxposs);
if numel(WasNeg) > 0
  PhotoC(WasNeg) = 0;  % actually should be negative: see next line
end
WasNeg = find(PhotoD > maxposs);
if numel(WasNeg) > 0
  PhotoD(WasNeg) = 0;  % actually should be negative: see next line
end
usethese = find((PhotoA >= 0) & (PhotoB >= 0) & (PhotoC >= 0) & (PhotoD >= 0) ...
  & (PhotoA < maxposs) & (PhotoB < maxposs) & (PhotoC < maxposs) & (PhotoD < maxposs)  ...
  & (PhotoAsh < maxposs) & (PhotoBsh < maxposs) & (PhotoCsh < maxposs) & (PhotoDsh < maxposs) ...
  & ((PhotoA+PhotoB+PhotoC+PhotoD) ~= 0));
% last because Arduiino writes all zeros if computation time overruns the integration time

npoints = numel(usethese);
PhotoA = PhotoA(usethese);
PhotoB = PhotoB(usethese);
PhotoC = PhotoC(usethese);
PhotoD = PhotoD(usethese);
PhotoAsh = PhotoAsh(usethese);
PhotoBsh = PhotoBsh(usethese);
PhotoCsh = PhotoCsh(usethese);
PhotoDsh = PhotoDsh(usethese);
homep = homep(usethese);
azimuth = azimuth(usethese);
Seconds = Seconds(usethese)*(millisscale/1000.);
MicroUsed = MicroUsed(usethese)*100.;    % factor 100 is because SASP Arduino code writes this in tenths of ms
%
% housekeeping type variables
%   for simplicity truncate to shortest of the rotating files; may lose a
%   fraction of a second of data
%
ishort = min(ione-1, itwo-1);
ishort = min(ishort, ithree-1);
ishort = min(ishort, ifour-1);
ishort = min(ishort, ifive-1);
Te = Te(1:ishort);
Lat = Lat(1:ishort);
Lon = Lon(1:ishort);
GPSHr = GPSHr(1:ishort);
MonthDay = MonthDay(1:ishort);
if MonthDay(ishort) > 1300 
  Year = 2000 + floor(MonthDay(ishort)/10000);
  MonthDay = mod(MonthDay,10000);
end
GPSReadSeconds = GPSReadSeconds(1:ishort);
HKSeconds = HKSeconds(1:ishort)*(millisscale/1000.);
Yaw = Yaw(1:ishort);
Pitch = Pitch(1:ishort);
Roll = Roll(1:ishort);
BaromPr = BaromPr(1:ishort);
BaromTe = BaromTe(1:ishort);
ModeFlag = floor(ModeFlag(1:ishort));
vscale = 4.95;
Te = (Te*(vscale/1023.)-0.5)*100.;  % convert digital bits to degrees 
%
% ------------------- fill and process azimuth, UDT --------------------
%
missaz = find(isnan(azimuth) > 0);   % azimuth has missing values due to rotating housekeeping variables
if numel(missaz) > 0
  previndx = (missaz - 1) > 1;
  azimuth(missaz) = azimuth(previndx) + (Seconds(missaz) - Seconds(previndx))*rotationrate;
  % this only fixes single missing values
end
% put times from internal clock onto GPS times 
%   uses a linear fit to account for clock drift, could also use interpolation or other methods
%   also assume that any lat and lon before GPS acquires are same as first GPS point
%
HadGPS = find((GPSReadSeconds > 0) & (MonthDay > 0));
nHadGPS = numel(HadGPS);
if (nHadGPS > 0) & (HadGPS(1) > 0) 
  MonthDay(1:HadGPS(1)-1) = MonthDay(HadGPS(1));
  % fails if go through midnight during period before GPS capture
  Lat(1:HadGPS(1)-1) = Lat(HadGPS(1));
  Lon(1:HadGPS(1)-1) = Lon(HadGPS(1));
end
Month = floor(MonthDay/100);
Day = MonthDay - Month*100;
if nHadGPS > 2 
  if MonthDay(ishort) > MonthDay(HadGPS(1))
     % NEED MATLAB CODE HERE to say how many days have passed given initial
     % and final month and day
    %TempDays = JULDAY(Month, MonthDay-Month, Year, 1, 0, 0)
    %GPSHr = GPSHr + 24.0*(TempDays - TempDays(1));
  end
  x = GPSHr(HadGPS);
  y = GPSReadSeconds(HadGPS);
  timefit = fit(x(:),y(:), 'poly1');
  timefitcoeff = coeffvalues(timefit);
else
  timefitcoeff(2) = 0;
  timefitcoeff(1) = 1.;
  %PRINT, 'Not enough GPS readings to compute times'
  %STOP
end
UDTofHK = (HKSeconds - timefitcoeff(2))/timefitcoeff(1); 
UDT = (Seconds - timefitcoeff(2))/timefitcoeff(1);
%
% -------------------------------------- BUILD MERGED DATA ----------------------------------------
%
% technical point: using short or long gate means a tiny change in time
%   time (UDT) is computed as a single time for each measurement, ignoring gate length
%   azimuths include the small change from gate length. This is why there are 4 of them: one color might be on short gate, another on long
%   gate length difference is about 0.3 degrees
%
shortgate = GateShArr(1)*1E-6;
longgate =  GateLgArr(1)*1E-6;
shorttimeoffset = -(longgate + 0.5*shortgate);
longtimeoffset = -0.5*longgate;
gateratio = longgate/shortgate;
PhotoAMerge = PhotoA;
PhotoBMerge = PhotoB;
PhotoCMerge = PhotoC;
PhotoDMerge = PhotoD;
AzimuthA = azimuth;
AzimuthB = azimuth;
AzimuthC = azimuth;
AzimuthD = azimuth;
UDT = UDT + longtimeoffset/3600.;

HiSigA = find(PhotoA > HiThres); 
LoSig = find(PhotoA <= HiThres);    % is there an easiser way in MATLAB to get complement indices?
%if numel(LoSig) > 1E6;   % robust fits are slow if too many points, so decimate
%  LoSig = LoSig(1:10:numel(LoSig));
%end
if numel(LoSig) > 0
  % find intercept between the integration times by fitting overlap data
  % slope is more accurate to use gateratio
  x = PhotoA(LoSig);
  y = PhotoAsh(LoSig);
  integfit = fit(x(:),y(:), 'poly1');    %  ROBUST FIT WOULD BE BETTER HERE (AND B, C, D)
  integfitcoeff = coeffvalues(integfit);
  AOffset = integfitcoeff(2);
  if numel(HiSigA) > 0
    PhotoAMerge(HiSigA) = (PhotoAsh(HiSigA)-AOffset)*gateratio;
    AzimuthA(HiSigA) = AzimuthA(HiSigA) + rotationrate*(shorttimeoffset - longtimeoffset);
  end
else
  PhotoAMerge = PhotoAsh;
end

HiSigB = find(PhotoB > HiThres); 
LoSig = find(PhotoB <= HiThres);  
if numel(LoSig) > 0
  x = PhotoB(LoSig);
  y = PhotoBsh(LoSig);
  integfit = fit(x(:),y(:), 'poly1');   
  integfitcoeff = coeffvalues(integfit);
  BOffset = integfitcoeff(2);
  if numel(HiSigB) > 0
    PhotoBMerge(HiSigB) = (PhotoBsh(HiSigB)-BOffset)*gateratio;
    AzimuthB(HiSigB) = AzimuthB(HiSigB) + rotationrate*(shorttimeoffset - longtimeoffset);
  end
else
  PhotoBMerge = PhotoBsh;
end

HiSigC = find(PhotoC > HiThres); 
LoSig = find(PhotoC <= HiThres);  
if numel(LoSig) > 0
  x = PhotoC(LoSig);
  y = PhotoCsh(LoSig);
  integfit = fit(x(:),y(:), 'poly1');   
  integfitcoeff = coeffvalues(integfit);
  COffset = integfitcoeff(2);
  if numel(HiSigC) > 0
    PhotoCMerge(HiSigC) = (PhotoCsh(HiSigC)-COffset)*gateratio;
    AzimuthC(HiSigC) = AzimuthC(HiSigC) + rotationrate*(shorttimeoffset - longtimeoffset);
  end
else
  PhotoCMerge = PhotoCsh;
end

HiSigD = find(PhotoD > HiThres); 
LoSig = find(PhotoD <= HiThres);  
if numel(LoSig) > 0
  x = PhotoD(LoSig);
  y = PhotoDsh(LoSig);
  integfit = fit(x(:),y(:), 'poly1');   
  integfitcoeff = coeffvalues(integfit);
  DOffset = integfitcoeff(2);
  if numel(HiSigD) > 0
    PhotoDMerge(HiSigD) = (PhotoDsh(HiSigD)-DOffset)*gateratio;
    AzimuthD(HiSigD) = AzimuthD(HiSigD) + rotationrate*(shorttimeoffset - longtimeoffset);
  end
else
  PhotoDMerge = PhotoDsh;
end
PhotoAMerge = PhotoAMerge - ConstOffsetA;   % subtract dark current from merged gate data
PhotoBMerge = PhotoBMerge - ConstOffsetB;
PhotoCMerge = PhotoCMerge - ConstOffsetC;
PhotoDMerge = PhotoDMerge - ConstOffsetD;
%
% -------------------- Short version of data on each HK time (about 1.5 s instead of 33 ms)
%
HKPhotoMax = zeros(ishort, 4);
HKPhotoAvg = HKPhotoMax;
for i = 1:RollingPts
  BaseIndices(i) = i;
end
for i = 1:ishort 
  TheseIndices = BaseIndices + max(GPSTimeIndices(i),1);
  TheseIndices = min(TheseIndices, npoints);
  HKPhotoMax(i, 1) = max(PhotoAMerge(TheseIndices));
  HKPhotoMax(i, 2) = max(PhotoBMerge(TheseIndices));
  HKPhotoMax(i, 3) = max(PhotoCMerge(TheseIndices));
  HKPhotoMax(i, 4) = max(PhotoDMerge(TheseIndices));
  HKPhotoAvg(i, 1) = sum(PhotoAMerge(TheseIndices))/RollingPts;
  HKPhotoAvg(i, 2) = sum(PhotoBMerge(TheseIndices))/RollingPts;
  HKPhotoAvg(i, 3) = sum(PhotoCMerge(TheseIndices))/RollingPts;
  HKPhotoAvg(i, 4) = sum(PhotoDMerge(TheseIndices))/RollingPts;
end
%
% -------------------------------------- CALCULATED PARAMETERS -----------------------------------------
%
% solar zenith
;
UDtForZenith = horzcat(UDT(1), UDTofHK, UDT(npoints));
% use housekeeping spacing for fewer calculations, then interpolate
% adding elements at end helps at endpoints
latforzenith = horzcat(Lat(1), Lat, Lat(ishort));
lonforzenith = horzcat(Lon(1), Lon, Lon(ishort));
SolarZenithsh = latforzenith;
SolarAzimuthsh = latforzenith;
for i = 1:numel(latforzenith)
  %sunpos = sunPositionsingle(latforzenith(i), lonforzenith(i), year, month(1), day(1), UDtForZenith[i]*3600.);
  %SolarZenithsh(i) = 90.0 - sunpos(1);
  %SolarAzimuthsh(i) = sunpos(2);
end
%
% --------- put into cells, MATLAB expert may have another way
%
nhk = ishort;
fastvar{1} = PhotoAMerge;
fastvar{2} = PhotoBMerge;
fastvar{3} = PhotoCMerge;
fastvar{4} = PhotoDMerge;
fastvar{5} = AzimuthA;
fastvar{6} = AzimuthB;
fastvar{7} = AzimuthC;
fastvar{8} = AzimuthD;
fastvar{9} = GPSHr;
fastvar{10} = homep;

hkvar{1} = UDTofHK;
hkvar{2} = Te;
hkvar{3} = Lat;
hkvar{4} = Lon;
hkvar{5} = Month;
hkvar{6} = Day;
hkvar{7} = Year;
hkvar{8} = Yaw;
hkvar{9} = Pitch;
hkvar{10} = Roll;
hkvar{11} = BaromPr;
hkvar{12} = BaromTe;
hkvar{13} = SolarZenithsh;
hkvar{14} = SolarAzimuthsh;
hkvar{15} = ModeFlag;
hkvar{16} = HKPhotoMax;
hkvar{17} = HKPhotoAvg;
 

% end of function
end

