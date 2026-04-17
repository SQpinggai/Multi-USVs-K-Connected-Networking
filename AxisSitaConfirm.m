function sita = AxisSitaConfirm(x,y,mode)
PI = pi;
sita = 0;
if mode == 1
    if(y == 0 && x >= 0)	sita = 0.0;
    end
    if(y == 0 && x < 0)	sita = PI;
    end
    if(y > 0 && x > 0)	sita = atan(y/x); end
    if(y > 0 && x == 0)	sita = PI/2; end
    if(y > 0 && x < 0)	sita = atan(y/x) + PI; end
    if(y < 0 && x > 0)	sita = atan(y/x); end
    if(y < 0 && x == 0)	sita = -PI/2; end
    if(y < 0 && x < 0)	sita = atan(y/x) - PI; end
    if(y == 0 && x == 0)	sita = 0; end

elseif mode == 2
    if(y == 0 && x >= 0)	sita = 0.0;end
    if(y == 0 && x < 0)	sita = PI;end
    if(y > 0 && x > 0)	sita = atan(y/x);end
    if(y > 0 && x == 0)	sita = PI/2;end
    if(y > 0 && x < 0)	sita = atan(y/x) + PI;end
    if(y < 0 && x > 0)	sita = atan(y/x) + 2*PI;end
    if(y < 0 && x == 0)	sita = 1.5 * PI;end
    if(y < 0 && x < 0)	sita = atan(y/x) + PI;end
    if(y == 0 && x == 0)sita = 0; end
end

end
