#include <iostream>
#include <list>
//#include <Windows.h>
#include "astar.h"

#define DEBUG
#define WIDTH	10
#define HEIGHT	10

/* Point Ŭ���� ���� �� ���� */
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

void findCPP(Map& map, unsigned startX, unsigned startY);
int doIBMotion(Map& map, std::list<Point*>& backTrackingList, unsigned* x, unsigned* y, int count);
Point findNearestPosition(std::list<Point*> backTrackingList, const Point& now);

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
			if (j == 0 || j == HEIGHT - 1 || i == 0 || i == WIDTH - 1) // �׵θ��� 1 ����
				testMap.setObject(i, j);
		}
	}
	testMap.printMap();
	
	int flag = 0, count = 0;
	
	findCPP(testMap, 1, 1);
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
int doIBMotion(Map& map, std::list<Point*> &backTrackingList, unsigned* x, unsigned* y, int count)
{
	while (1) {
		map.setMapData(*x, *y, count++);	// ������ ��� üũ
		for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ) {	// list �ݺ�
			if ((*iter)->x == *x && (*iter)->y == *y) {	// position�� ������ ������
				delete (*iter);	// �޸� ����
				iter = backTrackingList.erase(iter);	// 
			}
			else
				iter++;
		}
		
		if (map.getMapData(*x, *y + 1) == LOAD && map.getMapData(*x - 1, *y) == LOAD) { // �� , ��
			(*x)--;	// ��
		}
		else if (map.getMapData(*x, *y - 1) == LOAD && map.getMapData(*x - 1, *y) == LOAD) { // ��, ��
			(*x)--;	// ��
		}
		else if (map.getMapData(*x, *y - 1) == LOAD && map.getMapData(*x, *y + 1) == LOAD) { // ��, ��
			(*y)++;	// ��
		}
		else if (map.getMapData(*x, *y - 1) == LOAD) { // ��
			(*y)--;	// ��
		}
		else if (map.getMapData(*x, *y + 1) == LOAD) { // ��
			(*y)++;	// ��
		}
		else if (map.getMapData(*x - 1, *y) == LOAD) { // ��
			(*x)--;	// ��
		}
		else if (map.getMapData(*x + 1, *y) == LOAD) { // ��
			(*x)++;	// ��
		}
		else {
			break;
		}
	}
	
	return count;
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
	Point Nearest(now.x, now.y);	// ���� ������� ������ ����

#ifdef DEBUG
	puts("���� ��ġ���� backTrackingList �� ������ �Ÿ�");
#endif
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		double dist = sqrt(pow(abs((double)now.x - (*iter)->x), 2) + pow(abs((double)now.y - (*iter)->y), 2)); // ���� �� �� ���� �Ÿ� ���
		if (dist < minDistance) { // ���� ����� �Ÿ��� ���������� ������
			minDistance = dist; // minDist ������Ʈ
			Nearest.x = (*iter)->x;
			Nearest.y = (*iter)->y;
		}
#ifdef DEBUG
		printf("%d - %d, %d - %d\n", now.x, (*iter)->x, now.y, (*iter)->y);
		printf("[%f]\n", dist);
#endif
	}

#ifdef DEBUG
	printf("���� ������ ��ǥ: [%d, %d], �Ÿ�: [%f]\n", Nearest.x, Nearest.y, minDistance);
#endif

	return Nearest;
}

void findCPP(Map& map, unsigned startX, unsigned startY)
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
	int count = 2;
	
	Point tmp;
	Point Nearest;

	while (!backTrackingList.empty()) {
		tmp.x = x;
		tmp.y = y;
		Nearest = findNearestPosition(backTrackingList, tmp);
		printf("..[%d %d].. \n", Nearest.x, Nearest.y);
		// ���� ��ġ�κ��� ã�� Nearest ���� �ִܰŸ� ��� ���
		// Nearest �������� �̵�
		x = Nearest.x;
		y = Nearest.y;
		count = doIBMotion(map, backTrackingList, &x, &y, count);
		printf("%d\n", backTrackingList.empty());
	}
}