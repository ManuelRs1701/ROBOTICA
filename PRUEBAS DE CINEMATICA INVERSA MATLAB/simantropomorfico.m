clc
clear
l1=7;
l2=13;
l3=10;

l1=Revolute('d', l1 , 'a' , 0 , 'alpha', pi/2);
l2=Revolute('d', 0 , 'a' , l2 , 'alpha', 0);
l3=Revolute('d', 0 , 'a' , l3 , 'alpha', 0);
R=SerialLink([l1,l2,l3])
p=[10,0,10];
q=CIantropomorifoco(p)
R.plot([q(1),q(2),q(3)])

