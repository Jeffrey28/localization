function [fxL, fxR] = get_SV_focus(P)   % stereo vision의 초점(focus)이 맺히는 좌표(fxL, fxR) 계산
% P는 차량의 실제 위치 (또는 영상에서 quantization 된 포인트 클라우드에서 만들어진 타원 중심의 위치)
% 각 렌즈 위에 상이 맺히는 위치를 좌표로 변환(fxL, fxR)

Origin = [400; -1000; 0];   % stereo vision이 위치한 지점의 global 좌표
width = 0.07;   % stereo vision 사이 간격 (사람의 눈 간격과 동일하게 설정)
focus = 0.022;  % 각 렌즈와 상이 맺히는 곳까지의 거리
% stereo vision에서 각 camera의 원점 위치 (좌, 우)
OxL = [Origin(1)-width; Origin(2); Origin(3)];  % 좌측 카메라 원점
OxR = [Origin(1)+width; Origin(2); Origin(3)];  % 우측 카메라 원점

focus_offset = [OxL(2)+focus; OxR(2)+focus];    % 초점이 맺힐 위치의 global Y 좌표

t = (focus_offset(1)-OxL(2))/(P(2)-OxL(2));     % 좌측 벡터의 크기 파라미터
s = (focus_offset(2)-OxR(2))/(P(2)-OxR(2));     % 우측 벡터의 크기 파라미터

fxL = [OxL(1) + t*(P(1)-OxL(1)); focus_offset(1); OxL(3) + t*(P(3)-OxL(3))] + [randn*0.00000000001; 0; randn*0.00000000001];
fxR = [OxR(1) + s*(P(1)-OxR(1)); focus_offset(2); OxR(3) + s*(P(3)-OxR(3))] + [randn*0.00000000001; 0; randn*0.00000000001];

end