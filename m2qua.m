function [ qbn ] = m2qua( Cbn )
%% **************************************************************
%���ƣ�direction cosine Matrix to QUAternion
%���ܣ����������Ҿ���ת��Ϊ��̬��Ԫ��
%________________________________________________________________________
% ���룺
%       Cbn: ��bϵ��nϵ������ת������
% �����
%       qbn: rn = qbn*rb*qbn';
%_________________________________________________________________________
%���ߣ����������̴�ѧ �Զ���ѧԺ ���
%���ڣ�2020��8��14��
% ***********************************************************************
%%
% ����@ �����ߵ��㷨����ϵ������� P232
% ԭ��@  P246
% 
% ת����Ԫ�����ж�ֵ�ԣ�Ϊ��ȷ����Ԫ���и���Ԫ�ص������ţ�
% ��Ҫ����ȷ��һ��Ԫ��Ȼ���������Ԫ�أ���
% ʵ�ʼ����У�������ȷ������ֵ���Ԫ��(Ŀ����ȷ����Ԫ�ز�Ϊ0)������
% ��Ԫ�ط���ȡΪ�����ڼ�������Ԫ�ء�
C11 = Cbn(1, 1);    C12 = Cbn(1, 2);    C13 = Cbn(1, 3);
C21 = Cbn(2, 1);    C22 = Cbn(2, 2);    C23 = Cbn(2, 3);
C31 = Cbn(3, 1);    C32 = Cbn(3, 2);    C33 = Cbn(3, 3);

if C11 >= C22 + C33
    q1 = 0.5*sqrt(1 + C11 - C22 - C33);
    q0 = (C32 - C23)/(4*q1);
    q2 = (C12 + C21)/(4*q1);
    q3 = (C13 + C31)/(4*q1);
elseif C22 >= C11 + C33
    q2 = 0.5*sqrt(1 - C11 + C22 - C33);
    q0 = (C13 - C31)/(4*q2);
    q1 = (C12 + C21)/(4*q2);
    q3 = (C23 + C32)/(4*q2);
elseif C33 >= C11 + C22
    q3 = 0.5*sqrt(1 - C11 - C22 + C33);
    q0 = (C21 - C12)/(4*q3);
    q1 = (C13 + C31)/(4*q3);
    q2 = (C23 + C32)/(4*q3);
else
    q0 = 0.5*sqrt(1 + C11 + C22 + C33);
    q1 = (C32 - C23)/(4*q0);
    q2 = (C13 - C31)/(4*q0);
    q3 = (C21 - C12)/(4*q0); 
    
end

qbn = [q0, q1, q2, q3]';

end
