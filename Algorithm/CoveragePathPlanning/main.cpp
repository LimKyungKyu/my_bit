#include <iostream>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "astar.h"

#define CLEANED	128
#define RESIZEFACTOR	0.5

/* Position 클래스 선언 및 정의 */
class Position {
public:
	int x;
	int y;

	Position();
	Position(int _x, int _y);
};

Position::Position()
	: x(0), y(0) 
{

}

Position::Position(int _x, int _y)
	: x(_x), y(_y)
{

}

int doIBMotion(myMap& map, Position& now);
Position findNearestPosition(std::list<Position*> backTrackingList, const Position& now);
Position convertToResize(Position& origin, double resizeFactor);
Position convertToOrigin(Position& resize, double resizeFactor);

int main()
{
	// 1. astar
	//unsigned char* mapData = new unsigned char[100];
	//for (int i = 0; i < 100; ++i) {
	//	memset(mapData, LOAD, sizeof(unsigned char) * 100);
	//}
	//myMap mapp(10, 10, mapData); // 10x10 맵 생성
	//mapp.setObject(4, 3);
	//mapp.setObject(4, 4);
	//mapp.setObject(4, 5);
	//mapp.setObject(4, 6);
	//mapp.setObject(4, 7);
	//mapp.setObject(2, 5);
	//mapp.setObject(3, 5);
	//mapp.setObject(5, 5);

	//mapp.printMap();

	//Astar as;
	//as.setMap(mapp);

	//myNode* fin;
	//fin = as.findRoute(0, 0, 9, 9);
	//as.setPathToMap(fin);
	//as.printMapAll();

	//2. Coverage Path Planning
	cv::Mat test = cv::imread("matrix.bmp", cv::IMREAD_GRAYSCALE);
	std::cout << "자르기 전 OriginMap 사이즈: " << test.cols << ", " <<test.rows << std::endl;

	cv::Mat temp2;
	test(cv::Rect(50, 50, 120, 120)).copyTo(temp2);	// rect(x, y, width, height)
	std::cout << "자르고 난 후 OriginMap 사이즈: " << temp2.cols << ", " << temp2.rows << std::endl;

	cv::Mat temp3;
	cv::resize(temp2, temp3, cv::Size(0, 0), RESIZEFACTOR, RESIZEFACTOR, cv::INTER_AREA);
	std::cout << "자르고 난 후 resizeMap(testMap) 사이즈: " << temp3.cols << ", " << temp3.rows << std::endl;

	cv::Mat temp4;

	//myMap testMap(test.cols, test.rows, test.data);
	myMap originMap(temp2.cols, temp2.rows, temp2.data);
	myMap testMap(temp3.cols, temp3.rows, temp3.data);

	cv::Mat resultOrigin(temp2.rows, temp2.cols, CV_8UC1, originMap.getMapAddr());
	cv::Mat resultTest(temp3.rows, temp3.cols, CV_8UC1, testMap.getMapAddr());
	
	cv::imshow("origin", temp2);
	cv::imshow("test", temp3);

	Astar astar;
	myNode* fin;
	astar.setMap(testMap);

	std::list<Position> astarRoute;
	std::list<Position*> backTrackingList;
	for (int j = 0; j != testMap.getMapWidth(); ++j) {
		for (int i = 0; i != testMap.getMapHeight(); ++i) {
			if (testMap.getMapData(i, j) == LOAD) {
				Position* p1 = new Position(i, j);
				backTrackingList.push_back(p1);
			}
		}
	}
	Position nowOrigin(80, 80);
	Position nowResized;
	Position Nearest;

	nowResized = convertToResize(nowOrigin, RESIZEFACTOR);
	
	bool direct = false;
	int ret, count = 2;

	printf("\n청소 시작\n\n");
	while (!backTrackingList.empty()) {

		cv::imshow("resultOrigin", resultOrigin);
		cv::imshow("resultTest", resultTest);
		cv::waitKey(1);
		//map.setMapData(x, y, count++);	// 지나간 경로 체크, 지도에 count값 표시
		testMap.setMapData(nowResized.x, nowResized.y, CLEANED);	// 지나간 경로 체크, 지도에 count값 표시
		for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ) {	// list 반복
			if ((*iter)->x == nowResized.x && (*iter)->y == nowResized.y) {	// Position가 같은게 있으면
				delete (*iter);	// 메모리 해제
				iter = backTrackingList.erase(iter);	// list에서 삭제
			}
			else
				iter++;
		}
		// 갈수있는지 체크, 갈수있으면 x y 업데이트해줌
		ret = doIBMotion(testMap, nowResized);
		if (ret == 0) {	// 갈 곳이 없으면
			Nearest = findNearestPosition(backTrackingList, nowResized);
			// 가장 가까운 점 찾기
			// A* 써서 그 지점 갈 경로 찾기
			printf("시작 testMap 좌표: [%d, %d]\n", nowResized.x, nowResized.y); // 원래 좌표
			fin = astar.findRoute(nowResized.x, nowResized.y, Nearest.x, Nearest.y); // 경로 찾기
			while (fin->parent != nullptr) { // 저장된 노드로부터 경로 추출
				Position tmp(fin->xPos, fin->yPos);
				printf("testMap 좌표: [%d, %d]\n", fin->xPos, fin->yPos); // testMap 좌표 출력
				tmp = convertToOrigin(tmp, RESIZEFACTOR);	// originMap 좌표로 변환
				printf("originMap 좌표: [%d, %d]\n", tmp.x, tmp.y);
				// 여기서 좌표값 push_front로 배열에 넣어서 주행 쪽으로 던져줘야할듯 함
				astarRoute.push_front(tmp);
				fin = fin->parent;
			}
			printf("가까운길 찾기 완료 \n\n");
			//printf("..[%d %d].. \n", Nearest.x, Nearest.y);
			nowResized.x = Nearest.x;
			nowResized.y = Nearest.y;
		}
		// x y로 실제 이동 명령
		nowOrigin = convertToOrigin(nowResized, RESIZEFACTOR);
		originMap.setMapData(nowOrigin.x, nowOrigin.y, CLEANED);
	}
	printf("청소 끝! \n");
	cv::imshow("resultOrigin", resultOrigin);
	cv::imshow("resultTest", resultTest);
	cv::waitKey(0);
}

/* Origin 좌표로 부터 Resize된 좌표로 변환 */
Position convertToResize(Position& origin, double resizeFactor) {
	Position temp;

	temp.x = (int)round(origin.x * resizeFactor);
	temp.y = (int)round(origin.y * resizeFactor);

	return temp;
}

/* Resize된 좌표로 부터 Origin 좌표로 변환 */
Position convertToOrigin(Position& resize, double resizeFactor) {
	Position temp;

	temp.x = (int)round(resize.x / resizeFactor);
	temp.y = (int)round(resize.y / resizeFactor);

	return temp;
}

/* 
IB(Intellectual Boustrophedon) 동작
	동 = x++,
	서 = x--,
	남 = y++,
	북 = y--,
	'ㄹ' 자 형태로 남/북 동작하며 서쪽에 공간이 있으면 서쪽 우선 동작
*/
int doIBMotion(myMap& map, Position& now)
{
	int ret = 1;
	if (map.getMapData(now.x, now.y + 1) == LOAD && map.getMapData(now.x - 1, now.y) == LOAD) { // 남 , 서
		now.x--;	// 서
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD && map.getMapData(now.x - 1, now.y) == LOAD) { // 북, 서
		now.x--;	// 서
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD && map.getMapData(now.x, now.y + 1) == LOAD) { // 북, 남
		now.y++;	// 남
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD) { // 북
		now.y--;	// 북
	}
	else if (map.getMapData(now.x, now.y + 1) == LOAD) { // 남
		now.y++;	// 남
	}
	else if (map.getMapData(now.x - 1, now.y) == LOAD) { // 서
		now.x--;	// 서
	}
	else if (map.getMapData(now.x + 1, now.y) == LOAD) { // 동
		now.x++;	// 동
	}
	else {
		ret = 0;
	}

	return ret;
}

/* backTrackingList 에 있는 좌표들 중 now로부터 가장 가까운 점 찾기*/
Position findNearestPosition(std::list<Position*> backTrackingList, const Position& now)
{
#ifdef DEBUG
	puts("");
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		printf("[%d, %d]\n", (*iter)->x, (*iter)->y);
	}
#endif
	double minDistance = 10000;
	Position Nearest(now.x, now.y);	// 가장 가까운점 저장할 변수

#ifdef DEBUG
	puts("현재 위치에서 backTrackingList 내 점과의 거리");
#endif
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		//double dist = sqrt(pow(abs((double)now.x - (*iter)->x), 2) + pow(abs((double)now.y - (*iter)->y), 2)); // 점과 점 간 직선 거리 계산
		int dist = abs((int)(now.x - (*iter)->x)) + abs((int)(now.y - (*iter)->y));
		if (dist < minDistance) { // 새로 계산한 거리가 기존꺼보다 작으면
			minDistance = dist; // minDist 업데이트
			Nearest.x = (*iter)->x;
			Nearest.y = (*iter)->y;
		}
#ifdef DEBUG
		printf("%d - %d, %d - %d\n", now.x, (*iter)->x, now.y, (*iter)->y);
		printf("[%f]\n", dist);
#endif
	}

#ifdef DEBUG
	printf("제일 가까운거 좌표: [%d, %d], 거리: [%f]\n", Nearest.x, Nearest.y, minDistance);
#endif

	return Nearest;
}
