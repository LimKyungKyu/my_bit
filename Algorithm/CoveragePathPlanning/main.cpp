#include <iostream>
#include <list>
//#include <Windows.h>
#include "astar.h"

//#define DEBUG
#define WIDTH	10
#define HEIGHT	10

class Position {	// x y 좌표 저장 클래스, struct로 만들어도 상관없을듯
public:
	int x;
	int y;

	Position();
	Position(int _x, int _y);
	//bool operator==(Position& point);
};

Position::Position() {
	x = 0;
	y = 0;
}

Position::Position(int _x, int _y) {
	x = _x;
	y = _y;
}


void findCPP(const Map& map, unsigned startX, unsigned startY);
int doIBMotion(const Map& map, std::list<Position*>& backTrackingList, int* x, int* y, int count);
Position& findNearestPosition(std::list<Position*> backTrackingList, Position& now);

int main()
{
	Map testMap(10, 10);	// test 맵 생성
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
			if (j == 0 || j == HEIGHT - 1 || i == 0 || i == WIDTH - 1)
				testMap.setObject(i, j);
		}
	}
	testMap.printMap();
	
	int flag = 0, count = 0;
	
	findCPP(testMap, 4, 2);	// coverage path 찾기
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
int doIBMotion(const Map& map, std::list<Position*> &backTrackingList, int* x, int* y, int count)
{
	while (1) {
		map.data[*x][*y] = count++;	// 지나간 경로 체크
		for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ) {	// list 반복
			if ((*iter)->x == *x && (*iter)->y == *y) {	// position이 같은게 있으면
				delete (*iter);	// 메모리 해제
				iter = backTrackingList.erase(iter);	// 
			}
			else
				iter++;
		}

		if (map.data[*x][*y + 1] == LOAD && map.data[*x - 1][*y] == LOAD) { // 남 , 서
			(*x)--;	// 서
		}
		else if (map.data[*x][*y - 1] == LOAD && map.data[*x - 1][*y] == LOAD) { // 북, 서
			(*x)--;	// 서
		}
		else if (map.data[*x][*y - 1] == LOAD && map.data[*x][*y + 1] == LOAD) { // 북, 남
			(*y)++;	// 남
		}
		else if (map.data[*x][*y - 1] == LOAD) { // 북
			(*y)--;	// 북
		}
		else if (map.data[*x][*y + 1] == LOAD) { // 남
			(*y)++;	// 남
		}
		else if (map.data[*x - 1][*y] == LOAD) { // 서
			(*x)--;	// 서
		}
		else if (map.data[*x + 1][*y] == LOAD) { // 동
			(*x)++;	// 동
		}
		else {
			break;
		}
	}
	
	return count;
}

/* backTrackingList에서 가장 가까운 점 찾아서 반환 */
Position& findNearestPosition(std::list<Position*> backTrackingList, Position& now)
{
#ifdef DEBUG
	puts("");
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		printf("[%d, %d]\n", (*iter)->x, (*iter)->y);
	}
#endif
	double minDistance = 10000;
	int minX = now.x, minY = now.y;
	Position Nearest;

#ifdef DEBUG
	puts("현재 위치에서 backTrackingList 내 점과의 거리");
#endif
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		double dist = sqrt(pow(now.x - (*iter)->x, 2) + pow(now.y - (*iter)->y, 2));
		if (dist < minDistance) {
			minDistance = dist;
			minX = (*iter)->x;
			minY = (*iter)->y;
		}
#ifdef DEBUG
		printf("[%f]\n", dist);
#endif
	}

#ifdef DEBUG
	printf("제일 가까운거 좌표: [%d, %d], 거리: [%f]\n", minX, minY, minDistance);
#endif
	Nearest.x = minX;
	Nearest.y = minY;

	return Nearest;
}

/* coverage path 찾기 */
void findCPP(const Map& map, unsigned startX, unsigned startY)
{
	std::list<Position*> backTrackingList;	// 아직 청소 안한 영역 담아 둘 list 생성
	for (int j = 0; j != WIDTH; ++j) {
		for (int i = 0; i != HEIGHT; ++i) {
			if (map.data[i][j] == LOAD) {
				Position* p1 = new Position(i, j);
				backTrackingList.push_back(p1);
			}
		}
	}

	//// std::list<Position*>::iterator iter
	//for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
	//	printf("[%d, %d]\n", (*iter)->x, (*iter)->y);
	//}

	int x = startX, y = startY;
	int count = 2;
	
	Position tmp;
	Position test;

	while (!backTrackingList.empty()) {	// 청소안된 영역이 없을 때 까지
		tmp.x = x;
		tmp.y = y;
		test = findNearestPosition(backTrackingList, tmp);	// 현 위치에서부터 청소안된 가장 가까운 점 찾기
#ifdef DEBUG
		printf("..[%d %d].. \n", test.x, test.y);
#endif
		x = test.x;
		y = test.y;
		count = doIBMotion(map, backTrackingList, &x, &y, count);	// IB 동작 수행
#ifdef DEBUG
		printf("%d\n", backTrackingList.empty());
#endif
	}

}