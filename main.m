% Author: Sitan Huang
% 
% This program is Free Software licensed under WTFPL and distributed in the
% hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
% implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%
% You should have received a copy of the WTFPL Public License
% along with this program.  If not, see <http://www.wtfpl.net/about/>.
%
%         DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE (WTFPL)
%                   Version 2, December 2004
% 
% Copyright (C) 2023 Sitan Huang<sitan.huang@vanderbilt.edu>
%
% Everyone is permitted to copy and distribute verbatim or modified
% copies of this license document, and changing it is allowed as long
% as the name is changed.
% 
%            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
%   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
%
%  0. You just DO WHAT THE FUCK YOU WANT TO.
%

% create a segment with beta=160deg and some initial conditions
seg1 = Segment(160) ...
    .add_ic(  0,    0,    0,    0,  nan) ...
    .add_ic( 80,   10,  nan,  nan,  nan) ...
    .add_ic(160,    0,    0,    0,  nan);
%         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%           deg     s     v     a     j
%    (use nan to not specify an initial condition)

% Creates a dwell segment
seg2 = Segment.dwell(20); % beta = 20 deg

seg3 = Segment(160) ...
    .add_ic(  0,    0,    0,    0,  nan) ...
    .add_ic( 80,   20,  nan,  nan,  nan) ...
    .add_ic(160,    0,    0,    0,  nan);

seg4 = Segment.dwell(20);

cam = Cam() ...
    .add_seg(seg1) ...
    .add_seg(seg2) ...
    .add_seg(seg3) ...
    .solve() ... % Solve all segments
    .disp_results(); % Display segment coefficients

% Get the s, v, a, j values of all segments (unnormalized wrt. omega)
% and the corresponding thetas of each index
[thetas, s, v, a, j] = cam.raw(0.01); % 0.01 = resolution in degrees

% ... Perform Rho and Phi calculations here ...

% Generate x, y, z coordinates for Solidworks curve of the
% Cam Profile with specified resolution in degrees and base
% radius in unit length.
[x, y, z] = cam.cam_profile(30, 0.01);
% Write result to SSV to be imported to Solidworks
dlmwrite('P02_cam_profile.txt', ...
    [x' y' z'], ...
    'delimiter', ' ', ...
    'precision', 12); %#ok<DLMWT> 

% Create an SVAJ Chart object with 0.01 deg as graphing resolution
Chart(cam, 0.01) ...
    .graph(1); % Omega = 1 rad/s

