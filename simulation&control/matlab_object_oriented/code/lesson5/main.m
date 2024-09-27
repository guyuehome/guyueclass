%三位学生各自的5个成绩
stu1=[91,95,70,66,88];
stu2=[77,72,75,90,81];
stu3=[80,75,88,90,62];

stus=[stu1',stu2',stu3'];
o1=score(stu1);
o2=score(stu2);
o3=score(stu3,'Liming');

os=[o1,o2,o3];%串接方法

ss(1,5)=score(stu1);%直接声明

%%
si=sine;
sq=square(2,2);
ss=[si,sq];%异构数组