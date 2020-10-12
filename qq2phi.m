function [ phi ] = qq2phi( qbp, qbn )
%% **************************************************************
%���ƣ�Quaternion Quaternion to PHI
%���ܣ����㺬��������̬��Ԫ������ֵ֮���Ӧ�ĵ�Ч��תʸ��
%________________________________________________________________________
% ���룺
%       qbp: ����������̬��Ԫ��
%       qbn: ��ʵ��̬��Ԫ��
% �����
%       phi: nϵ��pϵ�ĵ�Ч��תʸ��
%_________________________________________________________________________
%���ߣ����������̴�ѧ �Զ���ѧԺ ���
%���ڣ�2020��10��3��
% ************************************************************************
%%
qerr = qmul(qbn, qconj(qbp));
phi = q2rv(qerr);

end
