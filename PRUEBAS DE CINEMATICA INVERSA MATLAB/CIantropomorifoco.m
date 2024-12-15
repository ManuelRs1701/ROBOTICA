function q=CIantropomorifoco(pos)
l1=7;
l2=13;
l3=10;
px=pos(1);
py=pos(2);
pz=pos(3);
D= (px^2+py^2+(pz-l1)^2-l2^2-l3^2)/(2*l2*l3);
theta1=atan2(py,px);
theta3=atan2(-sqrt(1-D^2),D);
theta2=atan2 (pz-l1, sqrt(px^2+py^2))-atan2(l3*sin(theta3),l2+l3*cos(theta3));
fprintf('Las variables articulares son: ')
%q=(theta1*180/pi;theta2*180/pi;theta3*180/pi)
q=[theta1;theta2;theta3]

end