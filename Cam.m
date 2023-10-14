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

%#ok<*AGROW> 
classdef Cam
    properties
        segments = Segment.empty
        coeffs
    end

    methods
        function cam = add_seg(cam, seg)
            % Append a segment to the segments list.
            cam.segments(end + 1) = seg;
        end

        function cam = solve(cam)
            % Solve all segments
            for i = 1:length(cam.segments)
                cam.segments(i) = cam.segments(i).solve;
            end
        end

        function cam = disp_results(cam)
            % Prints the segment coefficients
            start_angle = 0;
            for i = 1:length(cam.segments)
                end_angle = start_angle + cam.segments(i).beta;

                fprintf("Segment %i (%.3f to %.3f):\n", i, rad2deg(start_angle), rad2deg(end_angle))
                cam.segments(i) = cam.segments(i).solve;
                disp(cam.segments(i).coeffs)

                start_angle = end_angle;
            end
        end

        function [x, y, z] = cam_profile(cam, Rb, resolution)
            % Generate x, y, z coordinates for Solidworks curve of the
            % Cam Profile with specified resolution in degrees and base
            % radius in unit length.

            resolution = deg2rad(resolution);

            syms theta;
        
            Theta_vals = [];

            s_vals = [];

            start_angle = 0;
            for i = 1:length(cam.segments)
                beta   = cam.segments(i).beta;
                end_angle = start_angle + beta;

                theta_vals = start_angle:resolution:end_angle;
                Theta_vals = [Theta_vals, theta_vals];

                s_func = double(subs(cam.segments(i).s_func, theta, theta_vals));

                s_vals = [s_vals, s_func];

                start_angle = end_angle;
            end
        
            R_theta = Rb + s_vals;
            
            x = R_theta .* cos(Theta_vals);
            y = R_theta .* sin(Theta_vals);
            z = zeros(size(x));
        end
    end
end