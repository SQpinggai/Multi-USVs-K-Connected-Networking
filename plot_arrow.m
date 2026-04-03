function plot_arrow(Point1,Point2,color)

    x1 = Point1(1); y1 = Point1(2);
    x2 = Point2(1);y2 = Point2(2);
    distance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
    arrow_len = 1;
    arrow_width = 0.5;

    x = x2 - x1; y = y2 - y1;
    Sita =  AxisSitaConfirm(x,y,1);
%     if (x>0&&y>=0) 
% 			Sita=atan(y/x);
%         elseif (y>0&&x<=0) 
% 			Sita=0.5*pi+atan(-x/y);
%         elseif (x<0&&y<=0)
% 			Sita=atan(y/x)+pi;
%         elseif(x>=0&&y<0)
% 			Sita=atan(-x/y)+1.5*pi;
% 		else 
% 			Sita=0;
%     end
%         fprintf("sita=%f\n",Sita*180/pi);
    Point4=[0 0];Point5=[0 0];Point3=[0 0];
    
    Point4(1) = x1 + (distance - arrow_len) * cos(Sita);
    Point4(2) = y1 + (distance - arrow_len) * sin(Sita);
    
    Point3(1) = Point4(1) + arrow_width * sin(Sita);
    Point3(2) = Point4(2) - arrow_width * cos(Sita);
    
    Point5(1) = Point4(1) - arrow_width * sin(Sita);
    Point5(2) = Point4(2) + arrow_width * cos(Sita);
    dot=zeros(6,2);
    dot(1,1)=x1;  dot(1,2)=y1;
    dot(2,1) = Point4(1);  dot(2,2) = Point4(2);
    dot(3,1) = Point3(1);  dot(3,2) = Point3(2);
    dot(4,1) = x2;  dot(4,2) = y2;
    dot(5,1)=Point5(1);  dot(5,2)=Point5(2);
    dot(6,1)=Point4(1);  dot(6,2)=Point4(2);
    plot(dot(:,1),dot(:,2),color);
    fill(dot(:,1),dot(:,2),color);
end