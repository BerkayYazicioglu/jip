function [ h ] = drawArrow(fig,x,y,xlimits,ylimits,props )

xlim(xlimits)
ylim(ylimits)

h = annotation(fig, 'arrow');
set(h,'parent', gca, ...
    'position', [x(1),y(1),x(2)-x(1),y(2)-y(1)], ...
    'HeadLength', 5, 'HeadWidth', 5, 'HeadStyle', 'cback1', ...
    props{:} );

end
