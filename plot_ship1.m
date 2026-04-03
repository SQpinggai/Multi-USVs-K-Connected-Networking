function ship = plot_ship1(Nt,Et,kt,length,width,head)
   ship_width=width/2;ship_length=length/2;ship_head=head/2;
   ship=zeros(6,2);
   baseleft=zeros(1,2);
   baseleft(1,1)=Nt+ship_width*sin(kt);
   baseleft(1,2)=Et-ship_width*cos(kt);
   baseright=zeros(2,1);
   baseright(1,1)=Nt-ship_width*sin(kt);
   baseright(1,2)=Et+ship_width*cos(kt);
   ship(1,1)=baseleft(1,1)+ship_length*cos(kt);ship(1,2)=baseleft(1,2)+ship_length*sin(kt);
   ship(2,1)=Nt+ship_head*cos(kt);ship(2,2)=Et+ship_head*sin(kt);
   ship(3,1)=baseright(1,1)+ship_length*cos(kt);ship(3,2)=baseright(1,2)+ship_length*sin(kt);
   ship(4,1)=baseright(1,1)-ship_length*cos(kt);ship(4,2)=baseright(1,2)-ship_length*sin(kt);
   ship(5,1)=baseleft(1,1)-ship_length*cos(kt); ship(5,2)=baseleft(1,2)-ship_length*sin(kt);
   ship(6,1)=ship(1,1);ship(6,2)=ship(1,2);
end

