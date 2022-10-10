function [ h ] = drawArrow(fig,ax,x,y,xlimits,ylimits,props )

xlim(ax,xlimits)
ylim(ax,ylimits)

h = annotation(fig, 'arrow');
set(h,'parent', ax, ...
    'position', [x(1),y(1),x(2)-x(1),y(2)-y(1)], ...
    'HeadLength', 5, 'HeadWidth', 5, 'HeadStyle', 'cback1', ...
    props{:} );

end
