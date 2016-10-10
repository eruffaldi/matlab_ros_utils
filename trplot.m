%TRPLOT Draw a coordinate frame
%
% TRPLOT(T, OPTIONS) draws a 3D coordinate frame represented by the homogeneous 
% transform T (4x4).
%
% H = TRPLOT(T, OPTIONS) as above but returns a handle.
%
% TRPLOT(H, T) moves the coordinate frame described by the handle H to
% the pose T (4x4).
%
% TRPLOT(R, OPTIONS) as above but the coordinate frame is rotated about the
% origin according to the orthonormal rotation matrix R (3x3).
%
% H = TRPLOT(R, OPTIONS) as above but returns a handle.
%
% TRPLOT(H, R) moves the coordinate frame described by the handle H to
% the orientation R.
%
% Options::
% 'color',C          The color to draw the axes, MATLAB colorspec C
% 'noaxes'           Don't display axes on the plot
% 'axis',A           Set dimensions of the MATLAB axes to A=[xmin xmax ymin ymax zmin zmax]
% 'frame',F          The coordinate frame is named {F} and the subscript on the axis labels is F.
% 'text_opts', opt   A cell array of MATLAB text properties
% 'handle',H         Draw in the MATLAB axes specified by the axis handle H
% 'view',V           Set plot view parameters V=[az el] angles, or 'auto' 
%                    for view toward origin of coordinate frame
% 'length',s         Length of the coordinate frame arms (default 1)
% 'arrow'            Use arrows rather than line segments for the axes
% 'width', w         Width of arrow tips (default 1)
% 'thick',t          Thickness of lines (default 0.5)
% '3d'               Plot in 3D using anaglyph graphics
% 'anaglyph',A       Specify anaglyph colors for '3d' as 2 characters for 
%                    left and right (default colors 'rc'): chosen from
%                    r)ed, g)reen, b)lue, c)yan, m)agenta.
% 'dispar',D         Disparity for 3d display (default 0.1)
% 'text'             Enable display of X,Y,Z labels on the frame
% 'labels',L         Label the X,Y,Z axes with the 1st, 2nd, 3rd character of the string L
% 'rgb'              Display X,Y,Z axes in colors red, green, blue respectively
% 'rviz'             Display chunky rviz style axes
%
% Examples::
%
%       trplot(T, 'frame', 'A')
%       trplot(T, 'frame', 'A', 'color', 'b')
%       trplot(T1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
%       trplot(T1, 'labels', 'NOA');
%
%       h = trplot(T, 'frame', 'A', 'color', 'b');
%       trplot(h, T2);
%
% 3D anaglyph plot
%       trplot(T, '3d');
%
% Notes::
% - The 'rviz' option is equivalent to 'rgb', 'notext', 'noarrow', 
%   'thick', 5.
% - The arrow option requires the third party package arrow3 from File
%   Exchange.
% - The handle H is an hgtransform object. 
% - When using the form TRPLOT(H, ...) to animate a frame it is best to set 
%   the axis bounds.
% - The '3d' option requires that the plot is viewed with anaglyph glasses.
% - You cannot specify 'color' and '3d' at the same time.
%
% See also TRPLOT2, TRANIMATE.

%TODO:
% 'rviz', chunky RGB lines, no arrows

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% TODO
%  need to decide how to handle scaling
%  what does hold on mean?  don't touch scaling?

function hout = trplot(T, varargin)

    if isscalar(T) && ishandle(T)
        % trplot(H, T)
        H = T; T = varargin{1};
        if isrot(T)
            T = r2t(T);
        end
        set(H, 'Matrix', T);
        
        % for the 3D case retrieve the right hgtransform and set it
        hg2 = get(H, 'UserData');
        if ~isempty(hg2)
            set(hg2, 'Matrix', T);
        end
        
        return;
    end

    if size(T,3) > 1
        error('trplot cannot operate on a sequence');
    end
%    if ~ishomog(T) && ~isrot(T)
%        error('trplot operates only on transform (4x4) or rotation matrix (3x3)');
%    end
    
    opt.color = [];
    opt.rgb = false;
    opt.axes = true;
    opt.axis = [];
    opt.frame = [];
    opt.text_opts = [];
    opt.view = [];
    opt.width = 1;
    opt.arrow = false;
    opt.labels = 'XYZ';
    opt.handle = [];
    opt.anaglyph = 'rc';
    opt.d_3d = false;
    opt.dispar = 0.1;
    opt.thick = 0.5;
    opt.length = 1;
    opt.text = true;
    opt.lefty = false;
    opt.rviz = false;

    opt = tb_optparse(opt, varargin);
        
    if opt.rviz
        opt.thick = 5;
        opt.arrow = false;
        opt.rgb = true;
        opt.text = false;
    end

    if ~isempty(opt.color) && opt.d_3d
        error('cannot specify ''color'' and ''3d'', use ''anaglyph'' option');
    end
    if isempty(opt.color)
        opt.color = 'b';
    end
    if isempty(opt.text_opts)
        opt.text_opts = {};
    end
    
    if opt.d_3d
        opt.color = ag_color(opt.anaglyph(1));
    end
    
    if isempty(opt.axis)
        % determine some default axis dimensions
        
        % get the origin of the frame
        if isrot(T)
            c = [0 0 0];  % at zero for a rotation matrix
        else
            c = transl(T);    
        end
        
        d = 1.2;
        opt.axis = [c(1)-d c(1)+d c(2)-d c(2)+d c(3)-d c(3)+d];
        
    end
    
    % TODO: should do the 2D case as well
    
    if ~isempty(opt.handle)
        hax = opt.handle;
        hold(hax);
    else
        ih = ishold;
        if ~ih
            % if hold is not on, then clear the axes and set scaling
            cla
            if ~isempty(opt.axis)
                axis(opt.axis);
            end
            daspect([1 1 1]);
            
            if opt.axes
                xlabel( 'X');
                ylabel( 'Y');
                zlabel( 'Z');
                rotate3d on
            end
            new_plot = true;
        end
        hax = gca;
        hold on
    end
    % hax is the handle for the axis we will work with, either new or
    % passed by option 'handle'

    opt.text_opts = [opt.text_opts, 'Color', opt.color];


    hg = hgtransform('Parent', hax);


    % trplot( Q.R, fmt, color);
    if isrot(T)
        T = r2t(T);
    end

    % create unit vectors
    o =  [0 0 0]';
    x1 = opt.length*[1 0 0]';
    y1 = opt.length*[0 1 0]';
    if opt.lefty
        z1 = opt.length*[0 0 -1]';
    else
        z1 = opt.length*[0 0 1]';
    end
    
    % draw the axes
    
    mstart = [o o o]';
    mend = [x1 y1 z1]';

    if opt.rgb
        axcolors = {'r', 'g', 'b'};
    else
        axcolors = { opt.color, opt.color, opt.color};
    end
    
    if opt.arrow
%         % draw the 3 arrows
%         S = [opt.color num2str(opt.width)];
%         ha = arrow3(mstart, mend, S);
%         for h=ha'
%             set(h, 'Parent', hg);
%         end
          daspect([1,1,1])
          for i=1:3
              ha = arrow3(mstart(i,1:3), mend(i,1:3), [axcolors{i} num2str(opt.width)]);
              set(ha, 'Parent', hg);
          end
    else
        for i=1:3
            plot2([mstart(i,1:3); mend(i,1:3)], 'Color', axcolors{i}, ...
                'LineWidth', opt.thick, ...
                'Parent', hg);
        end
    end
    
    % label the axes
    if isempty(opt.frame)
        fmt = '%c';
    else
        fmt = sprintf('%%c_{%s}', opt.frame);
    end
    
    if opt.text
        % add the labels to each axis
        h = text(x1(1), x1(2), x1(3), sprintf(fmt, opt.labels(1)), 'Parent', hg);
        set(h, opt.text_opts{:});
        
        h = text(y1(1), y1(2), y1(3), sprintf(fmt, opt.labels(2)), 'Parent', hg);
        set(h, opt.text_opts{:});
        
        h = text(z1(1), z1(2), z1(3), sprintf(fmt, opt.labels(3)), 'Parent', hg);
        set(h, opt.text_opts{:});
    end
    
    % label the frame
    if ~isempty(opt.frame)
        h = text(o(1)-0.04*x1(1), o(2)-0.04*y1(2), o(3)-0.04*z1(3), ...
            ['\{' opt.frame '\}'], 'Parent', hg);
        set(h, 'VerticalAlignment', 'middle', ...
            'HorizontalAlignment', 'center', opt.text_opts{:});
    end
    
    if ~opt.axes
        set(gca, 'visible', 'off');
    end
    if ischar(opt.view) && strcmp(opt.view, 'auto')
        cam = x1+y1+z1;
        view(cam(1:3));
    elseif ~isempty(opt.view)
        view(opt.view);
    end
    if isempty(opt.handle) && ~ih
        grid on
        hold off
    end
    
    % now place the frame in the desired pose
    set(hg, 'Matrix', T);

    
    if opt.d_3d
        % 3D display.  The original axes are for the left eye, and we add 
        % another set of axes to the figure for the right eye view and
        % displace its camera to the right of that of that for the left eye.
        % Then we recursively call trplot() to create the right eye view.
        
        left = gca;
        right = axes;
        
        % compute the offset in world coordinates
        off = -t2r(view(left))'*[opt.dispar 0 0]';
        pos = get(left, 'CameraPosition');
        
        set(right, 'CameraPosition', pos+off');
        set(right, 'CameraViewAngle', get(left, 'CameraViewAngle'));
        set(right, 'CameraUpVector', get(left, 'CameraUpVector'));
        target = get(left, 'CameraTarget');
        set(right, 'CameraTarget', target+off');
        
        % set perspective projections
                set(left, 'Projection', 'perspective');
        set(right, 'Projection', 'perspective');
        
        % turn off axes for right view
        set(right, 'Visible', 'Off');
        
        % set color for right view
        hg2 = trplot(T, 'color', ag_color(opt.anaglyph(2)));
        
        % the hgtransform for the right view is user data for the left
        % view hgtransform, we need to change both when we rotate the 
        % frame.
        set(hg, 'UserData', hg2);
    end

    % optionally return the handle, for later modification of pose
    if nargout > 0
        hout = hg;
    end
end

function out = ag_color(c)

% map color character to an color triple, same as anaglyph.m

    % map single letter color codes to image planes
    switch c
    case 'r'
        out = [1 0 0];        % red
    case 'g'
        out = [0 1 0];        % green
    case 'b'
        % blue
        out = [0 0 1];
    case 'c'
        out = [0 1 1];        % cyan
    case 'm'
        out = [1 0 1];        % magenta
    case 'o'
        out = [1 1 0];        % orange
    end
end


%OPTPARSE Standard option parser for Toolbox functions
%
% OPTOUT = TB_OPTPARSE(OPT, ARGLIST) is a generalized option parser for
% Toolbox functions.  OPT is a structure that contains the names and
% default values for the options, and ARGLIST is a cell array containing
% option parameters, typically it comes from VARARGIN.  It supports options
% that have an assigned value, boolean or enumeration types (string or
% int).
%
% The software pattern is:
%
%       function(a, b, c, varargin)
%       opt.foo = false;
%       opt.bar = true;
%       opt.blah = [];
%       opt.choose = {'this', 'that', 'other'};
%       opt.select = {'#no', '#yes'};
%       opt = tb_optparse(opt, varargin);
%
% Optional arguments to the function behave as follows:
%   'foo'           sets opt.foo := true
%   'nobar'         sets opt.foo := false
%   'blah', 3       sets opt.blah := 3
%   'blah',{x,y}    sets opt.blah := {x,y}
%   'that'          sets opt.choose := 'that'
%   'yes'           sets opt.select := (the second element)
%
% and can be given in any combination.
%
% If neither of 'this', 'that' or 'other' are specified then opt.choose := 'this'.
% Alternatively if:
%        opt.choose = {[], 'this', 'that', 'other'};
% then if neither of 'this', 'that' or 'other' are specified then opt.choose := []
%
% If neither of 'no' or 'yes' are specified then opt.select := 1.
%
% Note:
% - That the enumerator names must be distinct from the field names.
% - That only one value can be assigned to a field, if multiple values
%   are required they must placed in a cell array.
% - To match an option that starts with a digit, prefix it with 'd_', so
%   the field 'd_3d' matches the option '3d'.
% - OPT can be an object, rather than a structure, in which case the passed
%   options are assigned to properties.
%
% The return structure is automatically populated with fields: verbose and
% debug.  The following options are automatically parsed:
%   'verbose'       sets opt.verbose := true
%   'verbose=2'     sets opt.verbose := 2 (very verbose)
%   'verbose=3'     sets opt.verbose := 3 (extremeley verbose)
%   'verbose=4'     sets opt.verbose := 4 (ridiculously verbose)
%   'debug', N      sets opt.debug := N
%   'showopt'       displays opt and arglist
%   'setopt',S      sets opt := S, if S.foo=4, and opt.foo is present, then
%                   opt.foo is set to 4.
%
% The allowable options are specified by the names of the fields in the
% structure opt.  By default if an option is given that is not a field of 
% opt an error is declared.  
%
% [OPTOUT,ARGS] = TB_OPTPARSE(OPT, ARGLIST) as above but returns all the
% unassigned options, those that don't match anything in OPT, as a cell
% array of all unassigned arguments in the order given in ARGLIST.
%
% [OPTOUT,ARGS,LS] = TB_OPTPARSE(OPT, ARGLIST) as above but if any
% unmatched option looks like a MATLAB LineSpec (eg. 'r:') it is placed in LS rather
% than in ARGS.
%


% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
%

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% Modifications by Joern Malzahn to support classes in addition to structs

function [opt,others,ls] = tb_optparse(in, argv)

    if nargin == 1
        argv = {};
    end

    if ~iscell(argv)
        error('RTB:tboptparse:badargs', 'input must be a cell array');
    end

    arglist = {};

    argc = 1;
    opt = in;
    
    if ~isfield(opt, 'verbose')
        opt.verbose = false;
    end
    if ~isfield(opt, 'debug')
        opt.debug = 0;
    end

    showopt = false;
    choices = [];

    while argc <= length(argv)
        % index over every passed option
        option = argv{argc};
        assigned = false;
        
        if isstr(option)

            switch option
            % look for hardwired options
            case 'verbose'
                opt.verbose = true;
                assigned = true;
            case 'verbose=2'
                opt.verbose = 2;
                assigned = true;
            case 'verbose=3'
                opt.verbose = 3;
                assigned = true;
            case 'verbose=4'
                opt.verbose = 4;
                assigned = true;
            case 'debug'
                opt.debug = argv{argc+1};
                argc = argc+1;
                assigned = true;
            case 'setopt'
                new = argv{argc+1};
                argc = argc+1;
                assigned = true;

                % copy matching field names from new opt struct to current one
                for f=fieldnames(new)'
                    if isfield(opt, f{1})
                        opt.(f{1}) = new.(f{1});
                    end
                end
            case 'showopt'
                showopt = true;
                assigned = true;

            otherwise
                % does the option match a field in the opt structure?
%                 if isfield(opt, option) || isfield(opt, ['d_' option])
%                if any(strcmp(fieldnames(opt),option)) || any(strcmp(fieldnames(opt),))
                 if isfield(opt, option) || isfield(opt, ['d_' option]) || isprop(opt, option)
                    
                    % handle special case if we we have opt.d_3d, this
                    % means we are looking for an option '3d'
                    if isfield(opt, ['d_' option]) || isprop(opt, ['d_' option])
                        option = ['d_' option];
                    end
                    
                    %** BOOLEAN OPTION
                    val = opt.(option);
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt.(option) = true;
                    else
                        %** OPTION IS ASSIGNED VALUE FROM NEXT ARG
                        % otherwise grab its value from the next arg
                        try
                            opt.(option) = argv{argc+1};
                        catch me
                            if strcmp(me.identifier, 'MATLAB:badsubscript')
                                error('RTB:tboptparse:badargs', 'too few arguments provided for option: [%s]', option);
                            else
                                rethrow(me);
                            end
                        end
                        argc = argc+1;
                    end
                    assigned = true;
                elseif length(option)>2 && strcmp(option(1:2), 'no') && isfield(opt, option(3:end))
                    %* BOOLEAN OPTION PREFIXED BY 'no'
                    val = opt.(option(3:end));
                    if islogical(val)
                        % a logical variable can only be set by an option
                        opt.(option(3:end)) = false;
                        assigned = true;
                    end
                else
                    % the option doesn't match a field name
                    % let's assume it's a choice type
                    %     opt.choose = {'this', 'that', 'other'};
                    %
                    % we need to loop over all the passed options and look
                    % for those with a cell array value
                    for field=fieldnames(opt)'
                        val = opt.(field{1});
                        if iscell(val)
                            for i=1:length(val)
                                if isempty(val{i})
                                    continue;
                                end
                                % if we find a match, put the final value
                                % in the temporary structure choices
                                %
                                % eg. choices.choose = 'that'
                                %
                                % so that we can process input of the form
                                %
                                %  'this', 'that', 'other'
                                %
                                % which should result in the value 'other'
                                if strcmp(option, val{i})
                                    choices.(field{1}) = option;
                                    assigned = true;
                                    break;
                                elseif val{i}(1) == '#' && strcmp(option, val{i}(2:end))
                                    choices.(field{1}) = i;
                                    assigned = true;
                                    break;
                                end
                            end
                            if assigned
                                break;
                            end
                        end
                    end
                end
            end % switch
        end
        if ~assigned
            % non matching options are collected
            if nargout >= 2
                arglist = [arglist argv(argc)];
            else
                if isstr(argv{argc})
                    error(['unknown options: ' argv{argc}]);
                end
            end
        end
        
        argc = argc + 1;
    end % while
    
    % copy choices into the opt structure
    if ~isempty(choices)
        for field=fieldnames(choices)'
           opt.(field{1}) = choices.(field{1});
        end
    end

    % if enumerator value not assigned, set the default value
    if ~isempty(in)
        for field=fieldnames(in)'
            if iscell(in.(field{1})) && iscell(opt.(field{1}))
                val = opt.(field{1});
                if isempty(val{1})
                    opt.(field{1}) = val{1};
                elseif val{1}(1) == '#'
                    opt.(field{1}) = 1;
                else
                    opt.(field{1}) = val{1};
                end
            end
        end
    end
                        
    if showopt
        fprintf('Options:\n');
        opt
        arglist
    end

    if nargout == 3
        % check to see if there is a valid linespec floating about in the
        % unused arguments
        for i=1:length(arglist)
            s = arglist{i};
            % get color
            [b,e] = regexp(s, '[rgbcmywk]');
            s2 = s(b:e);
            s(b:e) = [];
            
            % get line style
            [b,e] = regexp(s, '(--)|(-.)|-|:');
            s2 = [s2 s(b:e)];
            s(b:e) = [];
            
            % get marker style
            [b,e] = regexp(s, '[o\+\*\.xsd\^v><ph]');
            s2 = [s2 s(b:e)];
            s(b:e) = [];
            
            % found one
            if isempty(s)
                ls = arglist{i};
                arglist(i) = [];
                others = arglist;
                break;
            end
        end
        ls = [];
        others = arglist;
    elseif nargout == 2
        others = arglist;
    end

end

%ISROT Test if SO(3) rotation matrix
%
% ISROT(R) is true (1) if the argument is of dimension 3x3 or 3x3xN, else false (0).
%
% ISROT(R, 'valid') as above, but also checks the validity of the rotation
% matrix.
%
% Notes::
% - A valid rotation matrix has determinant of 1.
%
% See also ISHOMOG, ISVEC.



% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function h = isrot(r, dtest)

    d = size(r);
    if ndims(r) >= 2
        h =  all(d(1:2) == [3 3]);

        if h && nargin > 1
            h = abs(det(r) - 1) < eps;
        end

    else
        h = false;
    end

end

%TRANSL Create or unpack an SE3 translational transform
%
% Create a translational transformation matrix::
%
% T = TRANSL(X, Y, Z) is an SE(3) homogeneous transform (4x4) representing
% a pure translation of X, Y and Z.
%
% T = TRANSL(P) is an SE(3) homogeneous transform (4x4) representing a
% translation of P=[X,Y,Z]. If P (Mx3) it represents a sequence and T
% (4x4xM) is a sequence of homogeneous transforms such that T(:,:,i)
% corresponds to the i'th row of P.
%
% Unpack the translational part of a transformation matrix::
%
% P = TRANSL(T) is the translational part of a homogeneous transform T as a
% 3-element column vector.  If T (4x4xM) is a homogeneous transform
% sequence the rows of P (Mx3) are the translational component of the
% corresponding transform in the sequence.
%
% [X,Y,Z] = TRANSL(T) is the translational part of a homogeneous transform
% T as three components.  If T (4x4xM) is a homogeneous transform sequence
% then X,Y,Z (1xM) are the translational components of the corresponding
% transform in the sequence.
%
% Notes::
% - Somewhat unusually this function performs a function and its inverse.  An
%   historical anomaly.
%
% See also CTRAJ.



% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [t1,t2,t3] = transl(x, y, z)
    if nargin == 1
        if ishomog(x)
            if ndims(x) == 3
                % transl(T)  -> P, trajectory case
                if nargout == 1
                    t1 = squeeze(x(1:3,4,:))';
                elseif nargout == 3
                    t1 = squeeze(x(1,4,:))';
                    t2 = squeeze(x(2,4,:))';
                    t3 = squeeze(x(3,4,:))';
                end
            else
                % transl(T)  -> P
                if nargout == 1 || nargout == 0
                    t1 = x(1:3,4);
                elseif nargout == 3
                    t1 = x(1,4);
                    t2 = x(2,4);
                    t3 = x(3,4);
                end
                    
            end
        elseif length(x) == 3
            % transl(P) -> T
            t = x(:);
            t1 =    [eye(3)          t(:);
                0   0   0   1];
        else
            % transl(P) -> T, trajectory case
            n = numrows(x);
            t1 = repmat(eye(4,4), [1 1 n]);
            t1(1:3,4,:) = x';
        end    
    elseif nargin == 3
        % transl(x,y,z) -> T
        t = [x; y; z];
        t1 =    rt2tr( eye(3), t);
    end

end


%ISHOMOG Test if SE(3) homogeneous transformation
%
% ISHOMOG(T) is true (1) if the argument T is of dimension 4x4 or 4x4xN, else 
% false (0).
%
% ISHOMOG(T, 'valid') as above, but also checks the validity of the rotation
% sub-matrix.
%
% Notes::
% - The first form is a fast, but incomplete, test for a transform is SE(3).
% - Does not work for the SE(2) case.
%
% See also ISROT, ISVEC.



% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function h = ishomog(tr, rtest)
    d = size(tr);
    if ndims(tr) >= 2
        h =  all(d(1:2) == [4 4]);

        if h && nargin > 1
            h = abs(det(tr(1:3,1:3)) - 1) < eps;
        end

    else
        h = false;
    end

end


%PLOT2 Plot trajectories
%
% PLOT2(P) plots a line with coordinates taken from successive rows of P.  P
% can be Nx2 or Nx3.
%
% If P has three dimensions, ie. Nx2xM or Nx3xM then the M trajectories are
% overlaid in the one plot.
%
% PLOT2(P, LS) as above but the line style arguments LS are passed to plot.
%
% See also PLOT.

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
function h = plot2(p1, varargin)

    if ndims(p1) == 2
        if numcols(p1) == 3,
            hh = plot3(p1(:,1), p1(:,2), p1(:,3), varargin{:});
        else
            hh = plot(p1(:,1), p1(:,2), varargin{:});
        end
        if nargout == 1,
            h = hh;
        end
    else
        clf
        hold on
        for i=1:size(p1,2)
            plot2( squeeze(p1(:,i,:))' );
        end
    end

end

%NUMCOLS Number of columns in matrix
%
% NC = NUMCOLS(M) is the number of columns in the matrix M.
%
% Notes::
% - Readable shorthand for SIZE(M,2);
%
% See also NUMROWS, SIZE.


% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function c = numcols(m)
	c = size(m,2);

end