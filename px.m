function [potencia,rms] = px(x,li,ls)
    if li<0
        li=(-1)*li;
    end
    potencia=(1/(li+ls))*(int(x^2,-li/2,ls/2))
    format short;
    rms=sqrt(potencia);
    rms=vpa(rms,5)
    
    
 end