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
classdef Segment
    properties
        % These are not normalized to omega.
        s_func
        v_func
        a_func
        j_func
        coeffs
        beta
    end

    properties (GetAccess = private)
        ics = [0 nan nan nan nan]
    end

    methods (Static)
        function seg = dwell(beta)
            seg = Segment(beta) ...
                .add_ic(   0,      0,    nan,    nan,  nan) ...
                .add_ic(beta,      0,    nan,    nan,  nan);
        end
    end

    methods
        function seg = Segment(beta)
            % Create a new Segment object with a defined beta value.
            seg.beta = deg2rad(beta);
        end

        function seg = add_ic(seg, deg, s, v, a, j)
            % Add initial condition at angle=rad, s, v, a, j.
            % Use nan for unspecified initial conditions.
            seg.ics = [seg.ics; deg2rad(deg) s v a j];
            seg.ics
        end

        function seg = solve(seg)
            % Solves the segment's unnormalizied svaj functions and their
            % coefficients.
            [max_theta, deg] = seg.get_deg_internal();
            
            if max_theta ~= seg.beta
                error("Max_theta (%f rad) in initial conditions does not match the specified Beta (%f rad) value.", max_theta, seg.beta);
            end

            syms theta Beta_;

            c = sym('c', [1, deg + 1]); % order = degree + 1 = # of terms = # of IC
        
            % theta in radians
            s = sum(c .* (theta / Beta_).^(0:deg));

            eqns = [];

            for i = 1:size(seg.ics, 1)
                theta_val = seg.ics(i, 1);
                s_val = seg.ics(i, 2);
                v_val = seg.ics(i, 3);
                a_val = seg.ics(i, 4);
                j_val = seg.ics(i, 5);
        
                if ~isnan(s_val)
                    eqns = [eqns, subs(subs(s, theta, theta_val), Beta_, seg.beta) == s_val];
                end
                if ~isnan(v_val)
                    eqns = [eqns, subs(subs(diff(s, theta), theta, theta_val), Beta_, seg.beta) == v_val];
                end
                if ~isnan(a_val)
                    eqns = [eqns, subs(subs(diff(diff(s, theta), theta), theta, theta_val), Beta_, seg.beta) == a_val];
                end
                if ~isnan(j_val)
                    eqns = [eqns, subs(subs(diff(diff(diff(s, theta), theta), theta), theta, theta_val), Beta_, seg.beta) == j_val];
                end
            end
        
            seg.coeffs = solve(eqns);
        
            seg.s_func = subs(subs(s, seg.coeffs), Beta_, seg.beta);
            seg.v_func = diff(seg.s_func, theta);
            seg.a_func = diff(seg.v_func, theta);
            seg.j_func = diff(seg.a_func, theta);
        end
    end

    methods (Access = private)
        function [max_theta, deg] = get_deg_internal(seg)
            deg = -1;
            max_theta = 0;
            for i = 1:size(seg.ics, 1)
                max_theta = max(max_theta, seg.ics(i, 1));
                for j = 2:5
                    if ~isnan(seg.ics(i, j))
                        deg = deg + 1;
                    end
                end
            end
        end
    end
end