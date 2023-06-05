function [legendTitle] = tractorTrailerLegendLabels(plotArrow,tractor_label,trailer_label)

if plotArrow

    N=22;
    legendTitle = cell(1,N);
    for i = 1:N
        legendTitle{1,i} = '';
    end

    legendTitle{1,4} = tractor_label;
    legendTitle{1,14} = '$$\phi$$';
    legendTitle{1,15} = '$$\theta$$';
    legendTitle{1,17} = trailer_label;
    legendTitle{1,22} = '$$\psi$$';

else

    N = 19;
    egendTitle = cell(1,N);
    for i = 1:N
        legendTitle{1,i} = '';
    end

    legendTitle{1,4} = tractor_label;
    legendTitle{1,15} = trailer_label;
end

end
