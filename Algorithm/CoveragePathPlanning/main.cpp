#include <iostream>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "astar.h"

#define CLEANED	128
#define RESIZEFACTOR	0.5

/* Position Ŭ���� ���� �� ���� */
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
	//myMap mapp(10, 10, mapData); // 10x10 �� ����
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
	std::cout << "�ڸ��� �� OriginMap ������: " << test.cols << ", " <<test.rows << std::endl;

	cv::Mat temp2;
	test(cv::Rect(50, 50, 120, 120)).copyTo(temp2);	// rect(x, y, width, height)
	std::cout << "�ڸ��� �� �� OriginMap ������: " << temp2.cols << ", " << temp2.rows << std::endl;

	cv::Mat temp3;
	cv::resize(temp2, temp3, cv::Size(0, 0), RESIZEFACTOR, RESIZEFACTOR, cv::INTER_AREA);
	std::cout << "�ڸ��� �� �� resizeMap(testMap) ������: " << temp3.cols << ", " << temp3.rows << std::endl;

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

	printf("\nû�� ����\n\n");
	while (!backTrackingList.empty()) {

		cv::imshow("resultOrigin", resultOrigin);
		cv::imshow("resultTest", resultTest);
		cv::waitKey(1);
		//map.setMapData(x, y, count++);	// ������ ��� üũ, ������ count�� ǥ��
		testMap.setMapData(nowResized.x, nowResized.y, CLEANED);	// ������ ��� üũ, ������ count�� ǥ��
		for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ) {	// list �ݺ�
			if ((*iter)->x == nowResized.x && (*iter)->y == nowResized.y) {	// Position�� ������ ������
				delete (*iter);	// �޸� ����
				iter = backTrackingList.erase(iter);	// list���� ����
			}
			else
				iter++;
		}
		// �����ִ��� üũ, ���������� x y ������Ʈ����
		ret = doIBMotion(testMap, nowResized);
		if (ret == 0) {	// �� ���� ������
			Nearest = findNearestPosition(backTrackingList, nowResized);
			// ���� ����� �� ã��
			// A* �Ἥ �� ���� �� ��� ã��
			printf("���� testMap ��ǥ: [%d, %d]\n", nowResized.x, nowResized.y); // ���� ��ǥ
			fin = astar.findRoute(nowResized.x, nowResized.y, Nearest.x, Nearest.y); // ��� ã��
			while (fin->parent != nullptr) { // ����� ���κ��� ��� ����
				Position tmp(fin->xPos, fin->yPos);
				printf("testMap ��ǥ: [%d, %d]\n", fin->xPos, fin->yPos); // testMap ��ǥ ���
				tmp = convertToOrigin(tmp, RESIZEFACTOR);	// originMap ��ǥ�� ��ȯ
				printf("originMap ��ǥ: [%d, %d]\n", tmp.x, tmp.y);
				// ���⼭ ��ǥ�� push_front�� �迭�� �־ ���� ������ ��������ҵ� ��
				astarRoute.push_front(tmp);
				fin = fin->parent;
			}
			printf("������ ã�� �Ϸ� \n\n");
			//printf("..[%d %d].. \n", Nearest.x, Nearest.y);
			nowResized.x = Nearest.x;
			nowResized.y = Nearest.y;
		}
		// x y�� ���� �̵� ���
		nowOrigin = convertToOrigin(nowResized, RESIZEFACTOR);
		originMap.setMapData(nowOrigin.x, nowOrigin.y, CLEANED);
	}
	printf("û�� ��! \n");
	cv::imshow("resultOrigin", resultOrigin);
	cv::imshow("resultTest", resultTest);
	cv::waitKey(0);
}

/* Origin ��ǥ�� ���� Resize�� ��ǥ�� ��ȯ */
Position convertToResize(Position& origin, double resizeFactor) {
	Position temp;

	temp.x = (int)round(origin.x * resizeFactor);
	temp.y = (int)round(origin.y * resizeFactor);

	return temp;
}

/* Resize�� ��ǥ�� ���� Origin ��ǥ�� ��ȯ */
Position convertToOrigin(Position& resize, double resizeFactor) {
	Position temp;

	temp.x = (int)round(resize.x / resizeFactor);
	temp.y = (int)round(resize.y / resizeFactor);

	return temp;
}

/* 
IB(Intellectual Boustrophedon) ����
	�� = x++,
	�� = x--,
	�� = y++,
	�� = y--,
	'��' �� ���·� ��/�� �����ϸ� ���ʿ� ������ ������ ���� �켱 ����
*/
int doIBMotion(myMap& map, Position& now)
{
	int ret = 1;
	if (map.getMapData(now.x, now.y + 1) == LOAD && map.getMapData(now.x - 1, now.y) == LOAD) { // �� , ��
		now.x--;	// ��
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD && map.getMapData(now.x - 1, now.y) == LOAD) { // ��, ��
		now.x--;	// ��
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD && map.getMapData(now.x, now.y + 1) == LOAD) { // ��, ��
		now.y++;	// ��
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD) { // ��
		now.y--;	// ��
	}
	else if (map.getMapData(now.x, now.y + 1) == LOAD) { // ��
		now.y++;	// ��
	}
	else if (map.getMapData(now.x - 1, now.y) == LOAD) { // ��
		now.x--;	// ��
	}
	else if (map.getMapData(now.x + 1, now.y) == LOAD) { // ��
		now.x++;	// ��
	}
	else {
		ret = 0;
	}

	return ret;
}

/* backTrackingList �� �ִ� ��ǥ�� �� now�κ��� ���� ����� �� ã��*/
Position findNearestPosition(std::list<Position*> backTrackingList, const Position& now)
{
#ifdef DEBUG
	puts("");
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		printf("[%d, %d]\n", (*iter)->x, (*iter)->y);
	}
#endif
	double minDistance = 10000;
	Position Nearest(now.x, now.y);	// ���� ������� ������ ����

#ifdef DEBUG
	puts("���� ��ġ���� backTrackingList �� ������ �Ÿ�");
#endif
	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		//double dist = sqrt(pow(abs((double)now.x - (*iter)->x), 2) + pow(abs((double)now.y - (*iter)->y), 2)); // ���� �� �� ���� �Ÿ� ���
		int dist = abs((int)(now.x - (*iter)->x)) + abs((int)(now.y - (*iter)->y));
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
