function [fxL, fxR] = get_SV_focus(P)   % stereo vision�� ����(focus)�� ������ ��ǥ(fxL, fxR) ���
% P�� ������ ���� ��ġ (�Ǵ� ���󿡼� quantization �� ����Ʈ Ŭ���忡�� ������� Ÿ�� �߽��� ��ġ)
% �� ���� ���� ���� ������ ��ġ�� ��ǥ�� ��ȯ(fxL, fxR)

Origin = [400; -1000; 0];   % stereo vision�� ��ġ�� ������ global ��ǥ
width = 0.07;   % stereo vision ���� ���� (����� �� ���ݰ� �����ϰ� ����)
focus = 0.022;  % �� ����� ���� ������ �������� �Ÿ�
% stereo vision���� �� camera�� ���� ��ġ (��, ��)
OxL = [Origin(1)-width; Origin(2); Origin(3)];  % ���� ī�޶� ����
OxR = [Origin(1)+width; Origin(2); Origin(3)];  % ���� ī�޶� ����

focus_offset = [OxL(2)+focus; OxR(2)+focus];    % ������ ���� ��ġ�� global Y ��ǥ

t = (focus_offset(1)-OxL(2))/(P(2)-OxL(2));     % ���� ������ ũ�� �Ķ����
s = (focus_offset(2)-OxR(2))/(P(2)-OxR(2));     % ���� ������ ũ�� �Ķ����

fxL = [OxL(1) + t*(P(1)-OxL(1)); focus_offset(1); OxL(3) + t*(P(3)-OxL(3))] + [randn*0.00000000001; 0; randn*0.00000000001];
fxR = [OxR(1) + s*(P(1)-OxR(1)); focus_offset(2); OxR(3) + s*(P(3)-OxR(3))] + [randn*0.00000000001; 0; randn*0.00000000001];

end