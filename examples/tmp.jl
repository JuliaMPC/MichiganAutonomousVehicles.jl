using Plots

w = 3
h = 3
psi = 0
XQ = [-w/2 w/2 w/2 -w/2 -w/2]
YQ = [h/2 h/2 -h/2 -h/2 h/2]
P = [XQ;YQ]
ct = cos(psi)
st = sin(psi)
R = [ct -st;st ct]
P2 = R*P

plot()
function pltCurb(P2,w,h)
    scatter!((P2[1,:]+w,P2[2,:]+h),marker=(:circle,:black,0.,0.),fill=(0,1,:black),leg=true,grid=true,label="")
end

function intersectionPlot()
    pp = plot(0,leg=:false)

    # parameters TODO add to YAML file
    lw = 3
    w = 3
    h = 3
    psi = 0

    XQ = [-w/2 w/2 w/2 -w/2 -w/2]
    YQ = [h/2 h/2 -h/2 -h/2 h/2]
    P = [XQ;YQ]
    ct = cos(psi)
    st = sin(psi)
    R = [ct -st;st ct]
    P2 = R*P

    pltCurb(P2,w+lw/2,h+lw/2)
    pltCurb(P2,w+lw/2,-h-lw/2)
    pltCurb(P2,-w-lw/2,-h-lw/2)
    pltCurb(P2,-w-lw/2,h+lw/2)

    return pp
end

pp = intersectionPlot()
lim_size = 5
xlims!(-lim_size,lim_size)
ylims!(-lim_size,lim_size)
