darknet 오픈소스 이용한 특정 객체 인식 및 추적.

1. Jetson nano board <----TCP----> 2070 GPU PC (192.168.0.38)
			          Jetson TX2 board (192.168.0.xx)

    1. camera 이미지를		1. 소켓으로 이미지를 받아서
     소켓으로 전송			 darknet 으로 객체 인식
				2. 특정 클래스(현재는 사람class) 감지해서
				 일정 시간이상 감지 시 이미지 저장.
				3. 추적 모드일 때 강아지 class의 box 좌표
				 및 크기로 추적. (추적관련 제어는 파일로 저장)

2. Jetson nano board의 환경은 Linux여서 ip만 바꿔주면 되지만 darknet을 사용 할 환경은 Linux 또는 Windows임.

darknet 소스도 조금 다르고 socket 통신 구현도 조금 다름. 각각 프로그램 구현.

19/11/12.
이미지 소켓 통신에 있어 1M (608 x 608 x 3) 이미지 1장 당 200msec 전송 속도를 가짐.
고로 5 Mbps, 즉 1 FPS 나옴. 반응 속도가 매우 느림. 개선 필요

19/11/14.
이미지 소켓 통신 속도를 줄이기위해 보내기 전 jpeg 압축하여 보냄, quality 50으로 해서 10msec 속도 가짐.
GPU PC 의 Yolov3 경우 10 FPS, Yolov3-tiny 경우 15 FPS 나옴.
