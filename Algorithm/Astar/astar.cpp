#include <iostream>
#include "astar.h"

/* 
   ###################################################################################
   ################################  Node 클래스 정의  ################################
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

// Node 좌표값 비교, 같으면 1 아니면 0
bool Node::compNode(Node* node) {
	return (this->xPos == node->xPos && this->yPos == node->yPos);
}

// Node 초기화
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
   ################################   Map 클래스 정의  ################################
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

// 복사 생성자 (이거 안하면 Map 클래스를 인자로 받아서 처리할 때 얕은 복사 일어나서 나중에 소멸자 2번 부름)
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

// 복사 대입 연산자 재정의
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
// 외부 메모리 참고해서 Map 생성.
void Map::setMapData(unsigned width, unsigned height, int** _map_data) {
	xSize = width;
	ySize = height;

	data = _map_data;
}

// 맵에 장애물 설치
void Map::setObject(int _x, int _y) {
	data[_x][_y] = OBJECT;
}

// 전체 맵 출력
void Map::printMap() {
	printf("전체 맵 출력: %d x %d\n\n", xSize, ySize);
	for (int j = 0; j < ySize; ++j) {
		for (int i = 0; i < xSize; ++i) {
			printf("%2d ", data[i][j]);
		}
		puts("");
	}
}

// 갈 수 있는 좌표인지 아닌지. 갈 수 있으면 1, 아니면 0
bool Map::isWalkable(int _x, int _y) {
	if (_x >= xSize || _y >= ySize)
		return false;

	return (data[_x][_y] != OBJECT) ? true : false;
}

// 인자로 받은 Node 의 parent를 추적해서 PATH 그려줌.
void Map::setPath(Node* fin) {
	while (fin->parent != nullptr) {
		//printf("좌표: [%d, %d], F: %d, G: %d, H: %d\n", fin->xPos, fin->yPos, fin->fScore, fin->gScore, fin->hScore);
		data[fin->xPos][fin->yPos] = PATH;
		fin = fin->parent;
	}
	//printf("좌표: [%d, %d], F: %d, G: %d, H: %d\n", fin->xPos, fin->yPos, fin->fScore, fin->gScore, fin->hScore);
	data[fin->xPos][fin->yPos] = PATH;
}

/*
   ###################################################################################
   ###############################  pqueue 클래스 정의  ###############################
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

// 삽입
void pqueue::push(Node* const node) {
	que.push_back(node);
	size++;
}

// 정렬 (fScore 기준, 내림차순)
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

// 해당 좌표에 중복 노드 있는지 확인, 있다면 g 값 비교해서 더 작으면 g 값 및 f 값 갱신
void pqueue::pushNode(Node* const newNode) {
	for (int i = size - 1; i >= 0; i--) {
		if (que[i]->compNode(newNode)) { // 중복되는 노드가 있는지, 있다면
			if (que[i]->gScore > newNode->gScore) { // 새 노드가 g값이 더 작다면 g값 변경 및 f값 다시 계산
				que[i]->gScore = newNode->gScore;
				que[i]->fScore = que[i]->gScore + que[i]->fScore;
			}
			else {	// 새 노드가 g값이 더 높으면 삭제
				delete newNode;
				return;
			}
		}
	}	// 중복노드 없으면
	push(newNode);
}

// 큐 안에 같은게 있는지?
bool pqueue::isInQueue(Node* node) {
	for (std::vector<Node*>::iterator iter = que.begin(); iter != que.end(); ++iter) {
		if ((*iter)->compNode(node))
			return true;
	}
	return false;
}

// 맨 뒤에 (f값이 가장 작은 노드) 삭제
void pqueue::pop() {
	if (size != 0) {
		que.pop_back();
		size--;
	}
	else {
		std::cerr << "pqueue is empty\n";
	}
}

// 맨 뒤에 (f값이 가장 작은 노드) 노드 참조
Node* pqueue::top() {
	if (size != 0)
		return que.back();
	else
		return nullptr;
}

// 비었는지?
bool pqueue::empty() {
	return que.empty();
}

// 큐 안에 갯수.
unsigned int pqueue::getSize() {
	return size;
}


/*
   ###################################################################################
   ###############################  Astar 클래스 정의  ################################
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

// 맴버변수들 초기화
void Astar::init() {
	while (!openList.empty()) {
		openList.pop();
	}
	closeList.clear();
	start.initNode();
	finish.initNode();
	hasRoute = false;
}

// 시작 노드 설정
void Astar::setStart(unsigned int _x, unsigned int _y) {
	//start->initNode();
	if (!map.isWalkable(_x, _y)) {
		std::cerr << "This point is object\n";
		return;
	}
	start.xPos = _x;
	start.yPos = _y;
}

// 목표 노드 설정
void Astar::setFinish(unsigned int _x, unsigned int _y) {
	//start->initNode();
	if (!map.isWalkable(_x, _y)) {
		std::cerr << "This point is object\n";
		return;
	}
	finish.xPos = _x;
	finish.yPos = _y;
}

// 맵 설정
void Astar::setMap(int _x, int _y, int** _map_data) {
	map.setMapData(_x, _y, _map_data);
}
void Astar::setMap(const Map& _map) {
	map = _map;
}

// H 값 계산
int Astar::calcH(Node* const from, Node* const to) {
	int temp = 0;
	temp += std::abs((int)(to->xPos - from->xPos));
	temp += std::abs((int)(to->yPos - from->yPos));
	return temp * 10;
}

// G 값 계산
int Astar::calcG(Node* const from, int diag) {
	if (diag) //대각선일 때
		return from->parent->gScore + 14;
	else
		return from->parent->gScore + 10;
}

// F 값 계산
void Astar::calcF(Node* temp, int diag) {
	//if (diag)	// 대각선일 때
	//	temp->gScore = temp->parent->gScore + 14;
	//else		// 수평, 수직일 때
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

// 닫힌 목록에 노드가 있는지? 있으면 1, 없으면 0
bool Astar::isInCloseList(Node* node) {
	for (std::vector<Node*>::iterator iter = closeList.begin(); iter != closeList.end(); ++iter) {
		if ((*iter)->compNode(node))
			return true;
	}
	return false;
}

//// 현재 노드에 인접한 노드를 검사해 열린 목록에 넣음
//bool Astar::checkAround(Node* now) {
//	bool isThere = false;
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos, now->yPos - 1)) { // 북 체크
//	if (map.isWalkable(now->xPos, now->yPos - 1)) { // 북 체크
//		Node* tmp = new Node(now->xPos, now->yPos - 1, now); // 북쪽 노드 생성
//		if (!isInCloseList(tmp)) {
//			calcF(tmp, 0);
//			openList.pushNode(tmp);
//			isThere = true;
//		}
//		else
//			delete tmp;
//	}
//
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos + 1, now->yPos - 1)) { // 북동 체크
//	if (map.isWalkable(now->xPos + 1, now->yPos - 1)) { // 북동 체크
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
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos + 1, now->yPos)) { // 동 체크
//	if (map.isWalkable(now->xPos + 1, now->yPos)) { // 동 체크
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
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos + 1, now->yPos + 1)) { // 남동 체크
//	if (map.isWalkable(now->xPos + 1, now->yPos + 1)) { // 남동 체크
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
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos, now->yPos + 1)) { // 남 체크
//	if (map.isWalkable(now->xPos, now->yPos + 1)) { // 남 체크
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
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos - 1, now->yPos + 1)) { // 남서 체크
//	if (map.isWalkable(now->xPos - 1, now->yPos + 1)) { // 남서 체크
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
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos - 1, now->yPos)) { // 서 체크
//	if (map.isWalkable(now->xPos - 1, now->yPos)) { // 서 체크
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
//	//if (map.isWalkable(now->xPos, now->yPos, now->xPos - 1, now->yPos - 1)) { // 북서 체크
//	if (map.isWalkable(now->xPos - 1, now->yPos - 1)) { // 북서 체크
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

// 현재 노드에 인접한 노드를 검사해 열린 목록에 넣음
bool Astar::checkAround(Node* now, bool allowDiagonal, bool crossCorner) {
	bool isThere = false;
	bool s0 = false, s1 = false, s2 = false, s3 = false;
	bool d0 = false, d1 = false, d2 = false, d3 = false;

	if (map.isWalkable(now->xPos, now->yPos - 1)) { // 북(↑) 체크
		Node* tmp = new Node(now->xPos, now->yPos - 1, now); // 북쪽 노드 생성
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s0 = true;
			isThere = true;
		}
		else
			delete tmp;
	}

	if (map.isWalkable(now->xPos + 1, now->yPos)) { // 동(→) 체크
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

	if (map.isWalkable(now->xPos, now->yPos + 1)) { // 남(↓) 체크
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

	if (map.isWalkable(now->xPos - 1, now->yPos)) { // 서(←) 체크
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
	if (!allowDiagonal)	// 대각 이동 불가면 반환
		return isThere;

	if (!crossCorner) {	// 코너에서 대각 이동 x
		d0 = s3 && s0;
		d1 = s0 && s1;
		d2 = s1 && s2;
		d3 = s2 && s3;
	}
	else {	// 코너에서 대각 이동 o
		d0 = s3 || s0;
		d1 = s0 || s1;
		d2 = s1 || s2;
		d3 = s2 || s3;
	}

	if (d0 && map.isWalkable(now->xPos - 1, now->yPos - 1)) { // 북서(↖) 체크
		Node* tmp = new Node(now->xPos - 1, now->yPos - 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d1 && map.isWalkable(now->xPos + 1, now->yPos - 1)) { // 북동(↗) 체크
		Node* tmp = new Node(now->xPos + 1, now->yPos - 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d2 && map.isWalkable(now->xPos + 1, now->yPos + 1)) { // 남동(↘) 체크
		Node* tmp = new Node(now->xPos + 1, now->yPos + 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d3 && map.isWalkable(now->xPos - 1, now->yPos + 1)) { // 남서(↙) 체크
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

// 시작노드와 목표노드 설정해서 최단거리 경로 찾기
Node* Astar::findRoute(unsigned int fromX, unsigned int fromY, unsigned int toX, unsigned int toY) {
	// 0. 초기화
	init();
	setStart(fromX, fromY);
	setFinish(toX, toY);

	// 1. 시작노드에서 인접한 & 갈수있는 노드들 열린목록에 넣기
	closeList.push_back(&start);
	checkAround(&start, DIAGONAL, CROSSCORNER);

	Node* now;
	while (1) {
		// 2. 열린목록에서 가장 낮은 F 비용 노드를 현재노드로 설정 후 닫힌 노드에 삽입
		openList.sorting();
		now = openList.top();
		if (now == nullptr) { // 열린목록에 더이상 노드가 없다면
			hasRoute = false; // 길찾기 실패
			break;
		}
		openList.pop();
		closeList.push_back(now);
		if (isInCloseList(&finish)) { // 닫힌목록에 목표 노드가 있다면
			hasRoute = true; // 길찾기 성공
			break;
		}

		// 3. 현재노드에서 인접한 & 갈수있는 노드들 열린목록에 넣기
		checkAround(now, DIAGONAL, CROSSCORNER);
	}
	return closeList.back();
}