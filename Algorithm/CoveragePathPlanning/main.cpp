#include <iostream>
#include <list>
//#include <Windows.h>
#include "astar.h"

//#define DEBUG
#define WIDTH	10
#define HEIGHT	10

class Position {	// x y ��ǥ ���� Ŭ����, struct�� ���� ���������
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
	Map testMap(10, 10);	// test �� ����
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
	
	findCPP(testMap, 4, 2);	// coverage path ã��
	testMap.printMap();
}

/* 
IB(Intellectual Boustrophedon) ����
	�� = x++,
	�� = x--,
	�� = y++,
	�� = y--,
	'��' �� ���·� ��/�� �����ϸ� ���ʿ� ������ ������ ���� �켱 ����
*/
int doIBMotion(const Map& map, std::list<Position*> &backTrackingList, int* x, int* y, int count)
{
	while (1) {
		map.data[*x][*y] = count++;	// ������ ��� üũ
		for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ) {	// list �ݺ�
			if ((*iter)->x == *x && (*iter)->y == *y) {	// position�� ������ ������
				delete (*iter);	// �޸� ����
				iter = backTrackingList.erase(iter);	// 
			}
			else
				iter++;
		}

		if (map.data[*x][*y + 1] == LOAD && map.data[*x - 1][*y] == LOAD) { // �� , ��
			(*x)--;	// ��
		}
		else if (map.data[*x][*y - 1] == LOAD && map.data[*x - 1][*y] == LOAD) { // ��, ��
			(*x)--;	// ��
		}
		else if (map.data[*x][*y - 1] == LOAD && map.data[*x][*y + 1] == LOAD) { // ��, ��
			(*y)++;	// ��
		}
		else if (map.data[*x][*y - 1] == LOAD) { // ��
			(*y)--;	// ��
		}
		else if (map.data[*x][*y + 1] == LOAD) { // ��
			(*y)++;	// ��
		}
		else if (map.data[*x - 1][*y] == LOAD) { // ��
			(*x)--;	// ��
		}
		else if (map.data[*x + 1][*y] == LOAD) { // ��
			(*x)++;	// ��
		}
		else {
			break;
		}
	}
	
	return count;
}

/* backTrackingList���� ���� ����� �� ã�Ƽ� ��ȯ */
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
	puts("���� ��ġ���� backTrackingList �� ������ �Ÿ�");
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
	printf("���� ������ ��ǥ: [%d, %d], �Ÿ�: [%f]\n", minX, minY, minDistance);
#endif
	Nearest.x = minX;
	Nearest.y = minY;

	return Nearest;
}

/* coverage path ã�� */
void findCPP(const Map& map, unsigned startX, unsigned startY)
{
	std::list<Position*> backTrackingList;	// ���� û�� ���� ���� ��� �� list ����
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

	while (!backTrackingList.empty()) {	// û�Ҿȵ� ������ ���� �� ����
		tmp.x = x;
		tmp.y = y;
		test = findNearestPosition(backTrackingList, tmp);	// �� ��ġ�������� û�Ҿȵ� ���� ����� �� ã��
#ifdef DEBUG
		printf("..[%d %d].. \n", test.x, test.y);
#endif
		x = test.x;
		y = test.y;
		count = doIBMotion(map, backTrackingList, &x, &y, count);	// IB ���� ����
#ifdef DEBUG
		printf("%d\n", backTrackingList.empty());
#endif
	}

}