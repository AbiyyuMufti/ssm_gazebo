% CASEID  CRUCIFORM X Wing & TAIL CONFIGURATION (EX-MM40B3)
% $FLTCON                    
NALPHA  = 8.000; % Number of Alpha                                             
NMACH   = 8.000; % Number of Mach                                          
MACH    = [0.2, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.93];
REN     = 10000000; % Range                  
ALPHA   = [0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0];

% $REFQ
XCG  = 3.243; % Distance from nose to center of gravity.                                                             
ZCG  = 0.000; % Vertical locations of the center of gravity.
SREF    = 0.096; % Scalar denoting the reference area for the case.
RHR     = 260.0; 
LREF    = 0.350;
BLAYER  = 0.000;

% $AXIBOD
LNOSE   = 0.370; % Nose Lenght
DNOSE   = 0.350; % Nose Diameter at Base
BNOSE   = 0.0;   % Nose Bluntness radius
% TRUNC   = .TRUE.,                                                           
LCENTR  = 5.580; % Missile length
DCENTR  = 0.350;
DEXIT   = 0.310;

% $FINSET1
XLE     = [2.809,3.363];
NPANEL  = 4.000;
PHIF    = [45.0, 135.0, 225.0, 315.0];
SWEEP   = [34.0, 34.0, 34.0, 34.0];
STA     = 0.0;
CHORD   = [0.146,0.111];
SSPAN   = [0.175,0.550];

% $FINSET2                                                                       
XLE     = [4.546,4.657];
NPANEL  = 4.000;
PHIF    = [45.0,135.000,225.000,315.000];
SWEEP   = 0.0; % Sweepback angle at each span station.
STA     = 1.0; % Chord station used in measuring sweep: 
        % STA=0.0 is leading edge
        % STA=1.0 is trailing edge
SSPAN   = [0.175,0.471];  % Semi-span locations.
CHORD   = [0.2144,0.1379];% Panel chord at each semi-span location 
CFOC    = [0.1866,0.290]; % Flap chord to fin chord ratio at each span station 
