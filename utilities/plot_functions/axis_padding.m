function [axis_lims] = axis_padding(limit,center,padding_val)

% Add some padding to the axis limits
        padding = (limit(2) - limit(1))*padding_val;

        axis_lims(1) = center - padding;
        axis_lims(2) = center + padding;
        
end