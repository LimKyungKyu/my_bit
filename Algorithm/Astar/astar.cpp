#include <iostream>
#include "astar.h"

/* 
   ###################################################################################
   ################################  Node Ŭ���� ����  ################################
   ###################################################################################
*/

Node::Node() {
	xPos = 0;
	yPos = 0;
	fScore = 0;
	gScore = 0;
	hScore = 0;
	parent = nullptr;
}

Node::Node(unsigned int _x, unsigned int _y) {
	xPos = _x;
	yPos = _y;
	fScore = 0;
	gScore = 0;
	hScore = 0;
	parent = nullptr;
}

Node::Node(unsigned int _x, unsigned int _y, Node* _parent) {
	xPos = _x;
	yPos = _y;
	fScore = 0;
	gScore = 0;
	hScore = 0;
	parent = _parent;
}

// Node ��ǥ�� ��, ������ 1 �ƴϸ� 0
bool Node::compNode(Node* node) {
	return (this->xPos == node->xPos && this->yPos == node->yPos);
}

// Node �ʱ�ȭ
void Node::initNode() {
	xPos = 0;
	yPos = 0;
	fScore = 0;
	gScore = 0;
	hScore = 0;
	parent = nullptr;
}

/*
   ###################################################################################
   ################################   Map Ŭ���� ����  ################################
   ###################################################################################
*/
Map::Map() {
	xSize = 0;
	ySize = 0;
	data = nullptr;
}

Map::Map(unsigned width, unsigned height) {
	xSize = width;
	ySize = height;

	data = new int* [xSize];
	for (int i = 0; i < xSize; ++i) {
		data[i] = new int[ySize];
		memset(data[i], LOAD, sizeof(int) * ySize);
	}
}

// ���� ������ (�̰� ���ϸ� Map Ŭ������ ���ڷ� �޾Ƽ� ó���� �� ���� ���� �Ͼ�� ���߿� �Ҹ��� 2�� �θ�)
Map::Map(const Map& map) {
	xSize = map.xSize;
	ySize = map.ySize;
	
	data = new int* [xSize];
	for (int i = 0; i < xSize; ++i) {
		data[i] = new int[ySize];
	}

	for (int i = 0; i < xSize; ++i) {
		for (int j = 0; j < ySize; ++j) {
			data[i][j] = map.data[i][j];
		}
	}
}
Map::~Map() {
	delete data;
}

// ���� ���� ������ ������
Map& Map::operator=(const Map& map) {
	xSize = map.xSize;
	ySize = map.ySize;

	data = new int* [xSize];
	for (int i = 0; i < xSize; ++i) {
		data[i] = new int[ySize];
	}

	for (int i = 0; i < xSize; ++i) {
		for (int j = 0; j < ySize; ++j) {
			data[i][j] = map.data[i][j];
		}
	}
	return *this;
}
// �ܺ� �޸� �����ؼ� Map ����.
void Map::setMapData(unsigned width, unsigned height, int** _map_data) {
	xSize = width;
	ySize = height;

	data = _map_data;
}

// �ʿ� ��ֹ� ��ġ
void Map::setObject(int _x, int _y) {
	data[_x][_y] = OBJECT;
}

// ��ü �� ���
void Map::printMap() {
	printf("��ü �� ���: %d x %d\n\n", xSize, ySize);
	for (int j = 0; j < ySize; ++j) {
		for (int i = 0; i < xSize; ++i) {
			printf("%2d ", data[i][j]);
		}
		puts("");
	}
}

// �� �� �ִ� ��ǥ���� �ƴ���. �� �� ������ 1, �ƴϸ� 0
bool Map::isWalkable(int _x, int _y) {
	if (_x >= xSize || _y >= ySize)
		return false;

	return (data[_x][_y] != OBJECT) ? true : false;
}

// ���ڷ� ���� Node �� parent�� �����ؼ� PATH �׷���.
void Map::setPath(Node* fin) {
	while (fin->parent != nullptr) {
		//printf("��ǥ: [%d, %d], F: %d, G: %d, H: %d\n", fin->xPos, fin->yPos, fin->fScore, fin->gScore, fin->hScore);
		data[fin->xPos][fin->yPos] = PATH;
		fin = fin->parent;
	}
	//printf("��ǥ: [%d, %d], F: %d, G: %d, H: %d\n", fin->xPos, fin->yPos, fin->fScore, fin->gScore, fin->hScore);
	data[fin->xPos][fin->yPos] = PATH;
}

/*
   ###################################################################################
   ###############################  pqueue Ŭ���� ����  ###############################
   ###################################################################################
*/
pqueue::pqueue() {
	que.reserve(100);
	size = 0;
}
pqueue::pqueue(int _reserve) {
	que.reserve(_reserve);
	size = 0;
}
pqueue::~pqueue() {
	for (std::vector<Node*>::iterator iter = que.begin(); iter != que.end(); ++iter)
		delete (*iter);
}

// ����
void pqueue::push(Node* const node) {
	que.push_back(node);
	size++;
}

// ���� (fScore ����, ��������)
void pqueue::sorting() {
	for (unsigned i = 1; i < size; ++i) {
		for (unsigned j = 0; j < i; ++j) {
			if (que[j]->fScore < que[i]->fScore) {
				Node* tmp = que[i];
				que[i] = que[j];
				que[j] = tmp;
			}
		}
	}
}

// �ش� ��ǥ�� �ߺ� ��� �ִ��� Ȯ��, �ִٸ� g �� ���ؼ� �� ������ g �� �� f �� ����
void pqueue::pushNode(Node* const newNode) {
	for (int i = size - 1; i >= 0; i--) {
		if (que[i]->compNode(newNode)) { // �ߺ��Ǵ� ��尡 �ִ���, �ִٸ�
			if (que[i]->gScore > newNode->gScore) { // �� ��尡 g���� �� �۴ٸ� g�� ���� �� f�� �ٽ� ���
				que[i]->gScore = newNode->gScore;
				que[i]->fScore = que[i]->gScore + que[i]->fScore;
			}
			else {	// �� ��尡 g���� �� ������ ����
				delete newNode;
				return;
			}
		}
	}	// �ߺ���� ������
	push(newNode);
}

// ť �ȿ� ������ �ִ���?
bool pqueue::isInQueue(Node* node) {
	for (std::vector<Node*>::iterator iter = que.begin(); iter != que.end(); ++iter) {
		if ((*iter)->compNode(node))
			return true;
	}
	return false;
}

// �� �ڿ� (f���� ���� ���� ���) ����
void pqueue::pop() {
	if (size != 0) {
		que.pop_back();
		size--;
	}
	else {
		std::cerr << "pqueue is empty\n";
	}
}

// �� �ڿ� (f���� ���� ���� ���) ��� ����
Node* pqueue::top() {
	if (size != 0)
		return que.back();
	else
		return nullptr;
}

// �������?
bool pqueue::empty() {
	return que.empty();
}

// ť �ȿ� ����.
unsigned int pqueue::getSize() {
	return size;
}


/*
   ###################################################################################
   ###############################  Astar Ŭ���� ����  ################################
   ###################################################################################
*/
Astar::Astar()
	: map() 
{
	hasRoute = false;
}

Astar::Astar(const Map& _map, Node _start, Node _finish) {
	map = _map;
	start = _start;
	finish = _finish;
	hasRoute = false;
}

Astar::~Astar() {

}

// �ɹ������� �ʱ�ȭ
void Astar::init() {
	while (!openList.empty()) {
		openList.pop();
	}
	closeList.clear();
	start.initNode();
	finish.initNode();
	hasRoute = false;
}

// ���� ��� ����
void Astar::setStart(unsigned int _x, unsigned int _y) {
	//start->initNode();
	if (!map.isWalkable(_x, _y)) {
		std::cerr << "This point is object\n";
		return;
	}
	start.xPos = _x;
	start.yPos = _y;
}

// ��ǥ ��� ����
void Astar::setFinish(unsigned int _x, unsigned int _y) {
	//start->initNode();
	if (!map.isWalkable(_x, _y)) {
		std::cerr << "This point is object\n";
		return;
	}
	finish.xPos = _x;
	finish.yPos = _y;
}

// �� ����
void Astar::setMap(int _x, int _y, int** _map_data) {
	map.setMapData(_x, _y, _map_data);
}
void Astar::setMap(const Map& _map) {
	map = _map;
}

// H �� ���
int Astar::calcH(Node* const from, Node* const to) {
	int temp = 0;
	temp += std::abs((int)(to->xPos - from->xPos));
	temp += std::abs((int)(to->yPos - from->yPos));
	return temp * 10;
}

// G �� ���
int Astar::calcG(Node* const from, int diag) {
	if (diag) //�밢���� ��
		return from->parent->gScore + 14;
	else
		return from->parent->gScore + 10;
}

// F �� ���
void Astar::calcF(Node* temp, int diag) {
	//if (diag)	// �밢���� ��
	//	temp->gScore = temp->parent->gScore + 14;
	//else		// ����, ������ ��
	//	temp->gScore = temp->parent->gScore + 10;
	temp->gScore = calcG(temp, diag);
	temp->hScore = calcH(temp, &finish);
	temp->fScore = temp->gScore + temp->hScore;
}

void Astar::setObjectToMap(unsigned int _x, unsigned int _y) {
	map.setObject(_x, _y);
}

void Astar::printMapAll() {
	map.printMap();
}

void Astar::setPathToMap(Node* fin) {
	map.setPath(fin);
}

bool Astar::isInOpenList(Node* node) {
	return openList.isInQueue(node);
}

// ���� ��Ͽ� ��尡 �ִ���? ������ 1, ������ 0
bool Astar::isInCloseList(Node* node) {
	for (std::vector<Node*>::iterator iter = closeList.begin(); iter != closeList.end(); ++iter) {
		if ((*iter)->compNode(node))
			return true;
	}
	return false;
}

//// ���� ��忡 ������ ��带 �˻��� ���� ��Ͽ� ����
//bool Astar::checkAround(Node* now) {
//	bool isThere = false;
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos, now->yPos - 1)) { // �� üũ
//	if (map.isWalkable(now->xPos, now->yPos - 1)) { // �� üũ
//		Node* tmp = new Node(now->xPos, now->yPos - 1, now); // ���� ��� ����
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 0);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos + 1, now->yPos - 1)) { // �ϵ� üũ
//	if (map.isWalkable(now->xPos + 1, now->yPos - 1)) { // �ϵ� üũ
//		Node* tmp = new Node(now->xPos + 1, now->yPos - 1, now);
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 1);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos + 1, now->yPos)) { // �� üũ
//	if (map.isWalkable(now->xPos + 1, now->yPos)) { // �� üũ
//		Node* tmp = new Node(now->xPos + 1, now->yPos, now);
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 0);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos + 1, now->yPos + 1)) { // ���� üũ
//	if (map.isWalkable(now->xPos + 1, now->yPos + 1)) { // ���� üũ
//		Node* tmp = new Node(now->xPos + 1, now->yPos + 1, now);
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 1);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos, now->yPos + 1)) { // �� üũ
//	if (map.isWalkable(now->xPos, now->yPos + 1)) { // �� üũ
//		Node* tmp = new Node(now->xPos, now->yPos + 1, now);
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 0);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos - 1, now->yPos + 1)) { // ���� üũ
//	if (map.isWalkable(now->xPos - 1, now->yPos + 1)) { // ���� üũ
//		Node* tmp = new Node(now->xPos - 1, now->yPos + 1, now);
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 1);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos - 1, now->yPos)) { // �� üũ
//	if (map.isWalkable(now->xPos - 1, now->yPos)) { // �� üũ
//		Node* tmp = new Node(now->xPos - 1, now->yPos, now);
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 0);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos - 1, now->yPos - 1)) { // �ϼ� üũ
//	if (map.isWalkable(now->xPos - 1, now->yPos - 1)) { // �ϼ� üũ
//		Node* tmp = new Node(now->xPos - 1, now->yPos - 1, now);
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 1);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//	return isThere;
//}

// ���� ��忡 ������ ��带 �˻��� ���� ��Ͽ� ����
bool Astar::checkAround(Node* now, bool allowDiagonal, bool crossCorner) {
	bool isThere = false;
	bool s0 = false, s1 = false, s2 = false, s3 = false;
	bool d0 = false, d1 = false, d2 = false, d3 = false;

	if (map.isWalkable(now->xPos, now->yPos - 1)) { // ��(��) üũ
		Node* tmp = new Node(now->xPos, now->yPos - 1, now); // ���� ��� ����
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s0 = true;
			isThere = true;
		}
		else
			delete tmp;
	}

	if (map.isWalkable(now->xPos + 1, now->yPos)) { // ��(��) üũ
		Node* tmp = new Node(now->xPos + 1, now->yPos, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s1 = true;
			isThere = true;
		}
		else
			delete tmp;
	}

	if (map.isWalkable(now->xPos, now->yPos + 1)) { // ��(��) üũ
		Node* tmp = new Node(now->xPos, now->yPos + 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s2 = true;
			isThere = true;
		}
		else
			delete tmp;
	}

	if (map.isWalkable(now->xPos - 1, now->yPos)) { // ��(��) üũ
		Node* tmp = new Node(now->xPos - 1, now->yPos, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s3 = true;
			isThere = true;
		}
		else
			delete tmp;
	}
	if (!allowDiagonal)	// �밢 �̵� �Ұ��� ��ȯ
		return isThere;

	if (!crossCorner) {	// �ڳʿ��� �밢 �̵� x
		d0 = s3 && s0;
		d1 = s0 && s1;
		d2 = s1 && s2;
		d3 = s2 && s3;
	}
	else {	// �ڳʿ��� �밢 �̵� o
		d0 = s3 || s0;
		d1 = s0 || s1;
		d2 = s1 || s2;
		d3 = s2 || s3;
	}

	if (d0 && map.isWalkable(now->xPos - 1, now->yPos - 1)) { // �ϼ�(��) üũ
		Node* tmp = new Node(now->xPos - 1, now->yPos - 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d1 && map.isWalkable(now->xPos + 1, now->yPos - 1)) { // �ϵ�(��) üũ
		Node* tmp = new Node(now->xPos + 1, now->yPos - 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d2 && map.isWalkable(now->xPos + 1, now->yPos + 1)) { // ����(��) üũ
		Node* tmp = new Node(now->xPos + 1, now->yPos + 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d3 && map.isWalkable(now->xPos - 1, now->yPos + 1)) { // ����(��) üũ
		Node* tmp = new Node(now->xPos - 1, now->yPos + 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	return isThere;
}

// ���۳��� ��ǥ��� �����ؼ� �ִܰŸ� ��� ã��
Node* Astar::findRoute(unsigned int fromX, unsigned int fromY, unsigned int toX, unsigned int toY) {
	// 0. �ʱ�ȭ
	init();
	setStart(fromX, fromY);
	setFinish(toX, toY);

	// 1. ���۳�忡�� ������ & �����ִ� ���� ������Ͽ� �ֱ�
	closeList.push_back(&start);
	checkAround(&start, DIAGONAL, CROSSCORNER);

	Node* now;
	while (1) {
		// 2. ������Ͽ��� ���� ���� F ��� ��带 ������� ���� �� ���� ��忡 ����
		openList.sorting();
		now = openList.top();
		if (now == nullptr) { // ������Ͽ� ���̻� ��尡 ���ٸ�
			hasRoute = false; // ��ã�� ����
			break;
		}
		openList.pop();
		closeList.push_back(now);
		if (isInCloseList(&finish)) { // ������Ͽ� ��ǥ ��尡 �ִٸ�
			hasRoute = true; // ��ã�� ����
			break;
		}

		// 3. �����忡�� ������ & �����ִ� ���� ������Ͽ� �ֱ�
		checkAround(now, DIAGONAL, CROSSCORNER);
	}
	return closeList.back();
}