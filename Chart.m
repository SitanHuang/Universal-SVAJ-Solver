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
classdef Chart
    properties (GetAccess = private)
        segments = Segment.empty
        resolution
    end

    methods
        function chart = Chart(cam, resolution)
            % Create an SVAJ Chart object with angle resolution in degrees.
            chart.segments = cam.segments;
            chart.resolution = deg2rad(resolution);
        end

        function chart = graph(chart, omega)
            % Graph SVAJ chart with omega in rad/s

            syms theta;

            res = chart.resolution;

            Theta_vals = [];

            s_vals = [];
            v_vals = [];
            a_vals = [];
            j_vals = [];

            start_angle = 0;
            for i = 1:length(chart.segments)
                beta   = chart.segments(i).beta;
                end_angle = start_angle + beta;

                theta_vals = start_angle:res:end_angle;
                Theta_vals = [Theta_vals, theta_vals];

                s_func = double(subs(chart.segments(i).s_func, theta, theta_vals));
                v_func = double(subs(chart.segments(i).v_func, theta, theta_vals));
                a_func = double(subs(chart.segments(i).a_func, theta, theta_vals));
                j_func = double(subs(chart.segments(i).j_func, theta, theta_vals));

                s_vals = [s_vals, s_func];
                v_vals = [v_vals, v_func];
                a_vals = [a_vals, a_func];
                j_vals = [j_vals, j_func];

                start_angle = end_angle;
            end
           
            Theta_vals = rad2deg(Theta_vals);
            max_range = max(Theta_vals);
        
            figure;
        
            subplot(4,1,1);
            plot(Theta_vals, s_vals, 'b');
            title('Displacement (s) vs. Theta');
            ylabel('s (mm)');
            xlim([0 max_range])
            grid on;
            
            subplot(4,1,2);
            plot(Theta_vals, v_vals .* omega, 'r');
            title('Velocity (v) vs. Theta');
            ylabel('v (mm/s)');
            xlim([0 max_range])
            grid on;
            
            subplot(4,1,3);
            plot(Theta_vals, a_vals .* omega^2, 'g');
            title('Acceleration (a) vs. Theta');
            ylabel('a (mm/s^2)');
            xlim([0 max_range])
            grid on;
            
            subplot(4,1,4);
            plot(Theta_vals, j_vals .* omega^3, 'm');
            title('Jerk (j) vs. Theta');
            xlabel('Theta (degrees)');
            xlim([0 max_range])
            ylabel('j (mm/s^3)');
            grid on;
        end
    end
end