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
	bool operator<(const Position& pos) const;
};

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


int doIBMotion(myMap& map, Position& now);
Position findNearestPosition(std::set<Position>& backTrackingList, const Position& now);
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

	//myMap testMap(test.cols, test.rows, test.data);
	myMap originMap(temp2.cols, temp2.rows, temp2.data);
	myMap testMap(temp3.cols, temp3.rows, temp3.data);

	cv::Mat resultOrigin(temp2.rows, temp2.cols, CV_8UC1, originMap.getMapAddr());
	cv::Mat resultTest(temp3.rows, temp3.cols, CV_8UC1, testMap.getMapAddr());
	
	cv::namedWindow("origin", cv::WINDOW_NORMAL);
	cv::namedWindow("test", cv::WINDOW_NORMAL);
	cv::namedWindow("resultOrigin", cv::WINDOW_NORMAL);
	cv::namedWindow("resultTest", cv::WINDOW_NORMAL);

	cv::imshow("origin", temp2);
	cv::imshow("test", temp3);

	Astar astar;
	myNode* fin;
	astar.setMap(testMap);

	bool astarFlag = false;
	std::vector<Position> astarRoute;
	std::set<Position> backTrackingList;
	for (int j = 0; j != testMap.getMapWidth(); ++j) {
		for (int i = 0; i != testMap.getMapHeight(); ++i) {
			if (testMap.getMapData(i, j) == LOAD) {
				backTrackingList.insert(Position(i, j));
			}
		}
	}
	Position nowOrigin(80, 80);	// ������ǥ, ���� catographer���� localization ���� ����
	Position nowResized;
	Position Nearest;

	nowResized = convertToResize(nowOrigin, RESIZEFACTOR);
	

	int ret;
	std::set<Position>::iterator findPos;

	printf("\nû�� ����\n\n");
	while (!backTrackingList.empty()) {
		// darknet���� ���� ������ ���� �б�

		cv::imshow("resultOrigin", resultOrigin);
		cv::imshow("resultTest", resultTest);
		cv::waitKey(50);
		if (!astarFlag) {
			testMap.setMapData(nowResized.x, nowResized.y, CLEANED);	// ������ ��� üũ, ������ CLEANED�� ǥ��
			findPos = backTrackingList.find(nowResized); // ���� ��ǥ�� backTrackingList�� �ִ��� ã��
			if (findPos != backTrackingList.end()) { // �ִٸ�
				backTrackingList.erase(findPos); // backTrackingList���� ����
			}

			// ��濡 ���� �����ִ��� üũ, ���������� x y ������Ʈ����
			ret = doIBMotion(testMap, nowResized);
			switch (ret) {
			case 0:	// ���� ������
				// ���� ����� �� ã��
				Nearest = findNearestPosition(backTrackingList, nowResized);

				printf("���� testMap ��ǥ: [%d, %d]\n", nowResized.x, nowResized.y); // ���� ��ǥ
				// A* �Ἥ �� ���� �� ��� ã��
				fin = astar.findRoute(nowResized.x, nowResized.y, Nearest.x, Nearest.y); // ��� ã��
				while (fin->parent != nullptr) { // ����� ���κ��� ��� ����
					Position tmp(fin->xPos, fin->yPos);
					astarRoute.push_back(tmp);	// vector�� ����� ��ε� ����.
					fin = fin->parent;
				}
				
				// astarRoute�� ����� �� ���� ���� �̵��ϰ� �� ����ϴ� ������ �Ѿ�� flag
				astarFlag = true;

				//nowResized.x = Nearest.x;
				//nowResized.y = Nearest.y;
				break;
			case 1: // ����
				// (int)round(x + 1 / RESIZEFACTOR) �� �Ѱ��ֱ�
				break;
			case 2: // ����
				// (int)round(x - 1 / RESIZEFACTOR) �� �Ѱ��ֱ�
				break;
			case 3: // ����
				// (int)round(y - 1 / RESIZEFACTOR) �� �Ѱ��ֱ�
				break;
			case 4: // ����
				// (int)round(y + 1 / RESIZEFACTOR) �� �Ѱ��ֱ�
				break;
			default:
				break;
			}

			nowOrigin = convertToOrigin(nowResized, RESIZEFACTOR);
			originMap.setMapData(nowOrigin.x, nowOrigin.y, CLEANED);
		}
		else { // astar ��ã�� �� ��
			nowOrigin = convertToOrigin(nowResized, RESIZEFACTOR);
			originMap.setMapData(nowOrigin.x, nowOrigin.y, 200);
			if (!astarRoute.empty()) { // �Ⱥ������
				Position tmp = astarRoute.back();
				// (int)round((tmp.x - nowResized.x) / RESIZEFACTOR) x �� ��ȭ�� �Ѱ��ֱ�
				// (int)round((tmp.y - nowResized.y) / RESIZEFACTOR) y �� ��ȭ�� �Ѱ��ֱ�
				astarRoute.pop_back();
				printf("testMap ��ǥ : [%d, %d]\n", tmp.x, tmp.y);
				nowResized = tmp;
			}
			else { // �������
				astarFlag = false;
				printf("������ ã�� �Ϸ� \n\n");
			}
		}
	}
	printf("û�� ��! \n");
	cv::imshow("resultOrigin", resultOrigin);
	cv::imshow("resultTest", resultTest);
	cv::waitKey(0);

	cv::destroyAllWindows();
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
	�� = x++, return 1
	�� = x--, return 2
	�� = y++, return 3
	�� = y--, return 4
	�̵��Ұ� ��, return 0
	'��' �� ���·� ��/�� �����ϸ� ���ʿ� ������ ������ ���� �켱 ����
*/
int doIBMotion(myMap& map, Position& now)
{
	int ret;
	if (map.getMapData(now.x, now.y + 1) == LOAD && map.getMapData(now.x - 1, now.y) == LOAD) { // �� , ��
		now.x--;	// ��
		ret = 2;
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD && map.getMapData(now.x - 1, now.y) == LOAD) { // ��, ��
		now.x--;	// ��
		ret = 2;
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD && map.getMapData(now.x, now.y + 1) == LOAD) { // ��, ��
		now.y++;	// ��
		ret = 3;
	}
	else if (map.getMapData(now.x, now.y - 1) == LOAD) { // ��
		now.y--;	// ��
		ret = 4;
	}
	else if (map.getMapData(now.x, now.y + 1) == LOAD) { // ��
		now.y++;	// ��
		ret = 3;
	}
	else if (map.getMapData(now.x - 1, now.y) == LOAD) { // ��
		now.x--;	// ��
		ret = 2;
	}
	else if (map.getMapData(now.x + 1, now.y) == LOAD) { // ��
		now.x++;	// ��
		ret = 1;
	}
	else {
		ret = 0;
	}

	return ret;
}


/* backTrackingList �� �ִ� ��ǥ�� �� now�κ��� ���� ����� �� ã�� */
Position findNearestPosition(std::set<Position>& backTrackingList, const Position& now)
{
	double minDistance = 10000;
	Position Nearest(now.x, now.y);	// ���� ������� ������ ����

	for (auto iter = backTrackingList.begin(); iter != backTrackingList.end(); ++iter) {
		int dist = abs((int)(now.x - (*iter).x)) + abs((int)(now.y - (*iter).y));
		if (dist < minDistance) { // ���� ����� �Ÿ��� ���������� ������
			minDistance = dist; // minDist ������Ʈ
			Nearest.x = (*iter).x;
			Nearest.y = (*iter).y;
		}
	}

	return Nearest;
}
