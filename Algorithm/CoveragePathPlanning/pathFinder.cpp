#include <iostream>
#include <algorithm>
#include "pathFinder.h"


/*
   ###################################################################################
   ##############################  Position 클래스 정의  ##############################
   ###################################################################################
*/
Position::Position()
	: x(0), y(0)
{

}

Position::Position(int _x, int _y)
	: x(_x), y(_y)
{

}
bool Position::operator<(const Position& pos) const {
	if (x == pos.x)
		return (y < pos.y);
	else
		return (x < pos.x);
}

/*
   ###################################################################################
   #############################  pathFinder 클래스 정의  #############################
   ###################################################################################
*/
void PathFinder::setMoveInterval(int _move) {
	move = _move;
}

/* 주어진 맵을 전부 탐색하는 좌표 List 반환 */
std::list<Position> PathFinder::findCoveragePath(Position start, cv::Mat mapImage, int show){
	now = start;	// test 80, 80
	map.setMap(mapImage.cols, mapImage.rows, mapImage.data);
	cv::Mat resultOrigin(map.getMapHeight(), map.getMapWidth(), CV_8UC1, map.getMapAddr());

	if (show) {
		cv::namedWindow("resultOrigin", cv::WINDOW_NORMAL);
		cv::imshow("origin", mapImage);
	}
	
	for (int j = start.y % move; j < map.getMapHeight(); j += move) {
		for (int i = start.x % move; i < map.getMapWidth(); i += move) {
			if (map.getMapData(i, j) == LOAD) {
				backTrackingList.insert(Position(i, j));
			}
		}
	}

	Astar astar;
	myNode* fin;
	astar.setMap(map);

	int ret;
	Position Nearest;
	std::set<Position>::iterator findPos;
	std::list<Position> pathList;

	printf("\n경로탐색 시작\n\n");
	pathList.push_back(now);

	while (!backTrackingList.empty()) {
		if (show) {
			cv::imshow("resultOrigin", resultOrigin);
			cv::waitKey(50);
		}
		
		map.setMapData(now.x, now.y, CLEANED);

		findPos = backTrackingList.find(now);
		if (findPos != backTrackingList.end()) { // 있다면
			backTrackingList.erase(findPos); // backTrackingList에서 삭제
		}

		// 사방에 대해 갈수있는지 체크, 갈수있으면 x y 업데이트해줌
		ret = doIBMotion();
		if (ret == 0) { // 갈곳 없으면
			// 가장 가까운 점 찾기
			Nearest = findNearestPosition();

			// A* 써서 그 지점 갈 경로 찾기
			fin = astar.findRoute(now.x, now.y, Nearest.x, Nearest.y); // 경로 찾기
			while (fin->parent != nullptr) { // 저장된 노드로부터 경로 추출
				Position tmp(fin->xPos, fin->yPos);
				astarRoute.push_back(tmp);	// vector에 추출된 경로들 저장.
				fin = fin->parent;
			}
			// pathList에 저장
			for (auto it = astarRoute.rbegin(); it != astarRoute.rend(); ++it)
				pathList.push_back(*it);
			
			if (show) {
				printf("A* 시작 : [%d, %d]\n", now.x, now.y);
				for (auto it = astarRoute.rbegin(); it != astarRoute.rend(); ++it) {
					Position tmp = (*it);
					map.setMapData(tmp.x, tmp.y, 200);
					printf("A* 경로 좌표 : [%d, %d]\n", tmp.x, tmp.y);
				}
				printf("가까운길 찾기 완료 \n\n");
			}
			// astarRoute 초기화 및 현재 좌표 갱신
			astarRoute.clear();
			now = Nearest;

		}
		else {
			pathList.push_back(now);
		}
	}
	printf("경로탐색 끝! \n");
	if (show) {
		cv::imshow("resultOrigin", resultOrigin);
		cv::waitKey(0);
		cv::destroyAllWindows();
	}

	return pathList;
}


/*
IB(Intellectual Boustrophedon) 동작
	동 = x++, 
	서 = x--, 
	남 = y++, 
	북 = y--, 
	이동불가 시, return 0
	'ㄹ' 자 형태로 남/북 동작하며 서쪽에 공간이 있으면 서쪽 우선 동작
*/
int PathFinder::doIBMotion()
{
	int ret = 1;
	if (map.getMapData(now.x, now.y + move) == LOAD && map.getMapData(now.x - move, now.y) == LOAD) { // 남 , 서
		//now.x--;	// 서
		now.x -= move;
		
	}
	else if (map.getMapData(now.x, now.y - move) == LOAD && map.getMapData(now.x - move, now.y) == LOAD) { // 북, 서
		//now.x--;	// 서
		now.x -= move;
	}
	else if (map.getMapData(now.x, now.y - move) == LOAD && map.getMapData(now.x, now.y + move) == LOAD) { // 북, 남
		//now.y++;	// 남
		now.y += move;
	}
	else if (map.getMapData(now.x, now.y - move) == LOAD) { // 북
		//now.y--;	// 북
		now.y -= move;
	}
	else if (map.getMapData(now.x, now.y + move) == LOAD) { // 남
		//now.y++;	// 남
		now.y += move;
	}
	else if (map.getMapData(now.x - move, now.y) == LOAD) { // 서
		//now.x--;	// 서
		now.x -= move;
	}
	else if (map.getMapData(now.x + move, now.y) == LOAD) { // 동
		//now.x++;	// 동
		now.x += move;
	}
	else {
		ret = 0;
	}

	return ret;
}


/* backTrackingList 에 있는 좌표들 중 now로부터 가장 가까운 점 찾기 */
Position PathFinder::findNearestPosition()
{
	double minDistance = 10000;
	Position Nearest(now.x, now.y);	// 가장 가까운점 저장할 변수

	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		int dist = abs((int)(now.x - (*iter).x)) + abs((int)(now.y - (*iter).y));
		if (dist < minDistance) { // 새로 계산한 거리가 기존꺼보다 작으면
			minDistance = dist; // minDist 업데이트
			Nearest.x = (*iter).x;
			Nearest.y = (*iter).y;
		}
	}

	return Nearest;
}
