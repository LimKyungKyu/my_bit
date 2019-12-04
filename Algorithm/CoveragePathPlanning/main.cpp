#include <iostream>
#include <list>
#include "astar.h"

#define DEBUG
#define WIDTH	10
#define HEIGHT	10

/* Point 클래스 선언 및 정의 */
class Point {
public:
	unsigned x;
	unsigned y;

	Point();
	Point(unsigned _x, unsigned _y);
};

Point::Point() {
	x = 0;
	y = 0;
}

Point::Point(unsigned _x, unsigned _y) {
	x = _x;
	y = _y;
}

int doIBMotion(Map& map, unsigned* x, unsigned* y);
Point findNearestPosition(std::list<Point*> backTrackingList, const Point& now);
void doCleanCoveragePath(Map& map, unsigned startX, unsigned startY);

int main()
{
	Map testMap(10, 10);
	testMap.setObject(3, 3);
	testMap.setObject(3, 4);
	testMap.setObject(3, 5);
	testMap.setObject(4, 3);
	testMap.setObject(4, 4);
	testMap.setObject(4, 5);
	testMap.setObject(5, 3);
	testMap.setObject(5, 4);
	testMap.setObject(5, 5);
	for (int j = 0; j != HEIGHT; ++j) {
		for (int i = 0; i != WIDTH; ++i) {
			if (j == 0 || j == HEIGHT - 1 || i == 0 || i == WIDTH - 1) // 테두리에 1 생성
				testMap.setObject(i, j);
		}
	}
	testMap.printMap();
	
	doCleanCoveragePath(testMap, 1, 1);
	testMap.printMap();
}

/* 
IB(Intellectual Boustrophedon) 동작
	동 = x++,
	서 = x--,
	남 = y++,
	북 = y--,
	'ㄹ' 자 형태로 남/북 동작하며 서쪽에 공간이 있으면 서쪽 우선 동작
*/
int doIBMotion(Map& map, unsigned* x, unsigned* y)
{
	int ret = 1;
	if (map.getMapData(*x, *y + 1) == LOAD && map.getMapData(*x - 1, *y) == LOAD) { // 남 , 서
		(*x)--;	// 서
	}
	else if (map.getMapData(*x, *y - 1) == LOAD && map.getMapData(*x - 1, *y) == LOAD) { // 북, 서
		(*x)--;	// 서
	}
	else if (map.getMapData(*x, *y - 1) == LOAD && map.getMapData(*x, *y + 1) == LOAD) { // 북, 남
		(*y)++;	// 남
	}
	else if (map.getMapData(*x, *y - 1) == LOAD) { // 북
		(*y)--;	// 북
	}
	else if (map.getMapData(*x, *y + 1) == LOAD) { // 남
		(*y)++;	// 남
	}
	else if (map.getMapData(*x - 1, *y) == LOAD) { // 서
		(*x)--;	// 서
	}
	else if (map.getMapData(*x + 1, *y) == LOAD) { // 동
		(*x)++;	// 동
	}
	else {
		ret = 0;
	}

	return ret;
}
Point findNearestPosition(std::list<Point*> backTrackingList, const Point& now)
{
#ifdef DEBUG
	puts("");
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		printf("[%d, %d]\n", (*iter)->x, (*iter)->y);
	}
#endif
	double minDistance = 10000;
	Point Nearest(now.x, now.y);	// 가장 가까운점 저장할 변수

#ifdef DEBUG
	puts("현재 위치에서 backTrackingList 내 점과의 거리");
#endif
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		double dist = sqrt(pow(abs((double)now.x - (*iter)->x), 2) + pow(abs((double)now.y - (*iter)->y), 2)); // 점과 점 간 직선 거리 계산
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

void doCleanCoveragePath(Map& map, unsigned startX, unsigned startY)
{
	std::list<Point*> backTrackingList;
	for (int j = 0; j != WIDTH; ++j) {
		for (int i = 0; i != HEIGHT; ++i) {
			if (map.getMapData(i, j) == LOAD) {
				Point* p1 = new Point(i, j);
				backTrackingList.push_back(p1);
			}
		}
	}

	unsigned x = startX, y = startY;
	bool direct = false;
	int ret, count = 2;
	
	Point tmp;
	Point Nearest;

	while (!backTrackingList.empty()) {
		
		map.setMapData(x, y, count++);	// 지나간 경로 체크, 지도에 count값 표시
		for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ) {	// list 반복
			if ((*iter)->x == x && (*iter)->y == y) {	// Point가 같은게 있으면
				delete (*iter);	// 메모리 해제
				iter = backTrackingList.erase(iter);	// list에서 삭제
			}
			else
				iter++;
		}
		// 갈수있는지 체크, 갈수있으면 x y 업데이트해줌
		ret = doIBMotion(map, &x, &y);
		// x y로 실제 이동 명령
		if (ret == 0) {	// 갈 곳이 없으면
			tmp.x = x;
			tmp.y = y;
			Nearest = findNearestPosition(backTrackingList, tmp);
			// 가장 가까운 점 찾기
			// A* 써서 그 지점 갈 경로 찾기
			printf("..[%d %d].. \n", Nearest.x, Nearest.y);
			x = Nearest.x;
			y = Nearest.y;
		}
		//printf("%d\n", backTrackingList.empty());
	}

}