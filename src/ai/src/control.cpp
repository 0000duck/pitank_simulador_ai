#include "../include/ai/control.hpp"

void wallInfoReceived(const game_engine::WallInfoArray& msg){
    w_array = msg;
}

void uiStateReceived(const game_engine::UIState& msg){
    uistate.aiGame = msg.aiGame;
    uistate.gameStart = msg.gameStart;
    uistate.paused = msg.paused;
    uistate.seconds = msg.seconds;
    uistate.setupStart = msg.setupStart;
    uistate.teamGame = msg.teamGame;
    uistate.simGame = msg.simGame;
}

void robotDescriptionReceived(const game_engine::RobotDescriptionArray& msg){
    if(robot.empty()){
        for (int i=0;i<msg.robot.size();i++){
            r.addr0 = msg.robot[i].addr0;
            r.addr1 = msg.robot[i].addr1;
            r.angle = msg.robot[i].angle;
            r.autonomous_drive = msg.robot[i].autonomous_drive;
            r.collisionFlag = msg.robot[i].collisionFlag;
            r.collisionStateVar = msg.robot[i].collisionStateVar;
            r.damage = msg.robot[i].damage;
            r.height = msg.robot[i].height;
            r.immobilized = msg.robot[i].immobilized;
            r.kills = msg.robot[i].kills;
            r.tagId = msg.robot[i].tagId;
            r.teamId = msg.robot[i].teamId;
            r.threadIsRunning = msg.robot[i].threadIsRunning;
            r.vel1 = msg.robot[i].vel1;
            r.vel2 = msg.robot[i].vel2;
            r.x = msg.robot[i].x;
            r.y = msg.robot[i].y;
            r.classification = msg.robot[i].classification;

            robot.push_back(r);
        }
    }
    else{
        for (int i=0;i<msg.robot.size();i++){
            robot[i].addr0 = msg.robot[i].addr0;
            robot[i].addr1 = msg.robot[i].addr1;
            robot[i].angle = msg.robot[i].angle;
            robot[i].autonomous_drive = msg.robot[i].autonomous_drive;
            robot[i].collisionFlag = msg.robot[i].collisionFlag;
            robot[i].collisionStateVar = msg.robot[i].collisionStateVar;
            robot[i].damage = msg.robot[i].damage;
            robot[i].height = msg.robot[i].height;
            robot[i].immobilized = msg.robot[i].immobilized;
            robot[i].kills = msg.robot[i].kills;
            robot[i].tagId = msg.robot[i].tagId;
            robot[i].teamId = msg.robot[i].teamId;
            robot[i].threadIsRunning = msg.robot[i].threadIsRunning;
            robot[i].vel1 = msg.robot[i].vel1;
            robot[i].vel2 = msg.robot[i].vel2;
            robot[i].x = msg.robot[i].x;
            robot[i].y = msg.robot[i].y;
            robot[i].classification = msg.robot[i].classification;
        }
    }
}

double normDegrees180(double angle){
    angle = fmod(angle + 180,360);
    if (angle < 0)
        angle += 360;
    return angle - 180;
}

void GoToXY(int id, int xf, int yf){
    if((robot[id].collisionFlag == false) && (robot[id].autonomous_drive == false)){
        double sign = cos(robot[id].angle * PI / 180)*(yf-robot[id].y)-sin(robot[id].angle * PI / 180)*(xf-robot[id].x);
        switch (state_gotoxy){
            case ROTATE:
                if(sqrt((xf-robot[id].x)*(xf-robot[id].x)+(yf-robot[id].y)*(yf-robot[id].y)) < ERRO_DIST){
                    state_gotoxy = END;
                }
                else if(abs(normDegrees180(diffAngle((atan2(yf-robot[id].y,xf-robot[id].x) * 180 / PI),robot[id].angle))) < MAX_ETF){
                    stop(id);
                    state_gotoxy = TRANSL;
                }
                else if(sign < 0){
                    move_left(id);
                }
                else if(sign >= 0){
                    move_right(id);
                }
            break;
            case TRANSL:
                if(abs(normDegrees180(diffAngle((atan2(yf-robot[id].y,xf-robot[id].x) * 180 / PI),robot[id].angle))) > MAX_ETF + HIST_ETF){
                    state_gotoxy = ROTATE;
                }
                else if(sqrt((xf-robot[id].x)*(xf-robot[id].x)+(yf-robot[id].y)*(yf-robot[id].y)) < ERRO_DIST){
                    state_gotoxy = END;
                }
                else{
                    move_forward(id,normDegrees180(diffAngle((atan2(yf-robot[id].y,xf-robot[id].x) * 180 / PI),robot[id].angle)));
                }
            break;
        }
        if (state_gotoxy == END){
            stop(id);
            end_gotoxy = true;
        }
    }
}

void rotateToEnemy(int id, int x_e, int y_e){
    while(1){
            double sign = cos(robot[id].angle * PI / 180)*(y_e-robot[id].y)-sin(robot[id].angle * PI / 180)*(x_e-robot[id].x);
            if(abs(normDegrees180(diffAngle((atan2(y_e-robot[id].y,x_e-robot[id].x) * 180 / PI),robot[id].angle))) < MAX_ETF){
                break;
            }
            else if(sign < 0){
                move_left(id);
            }
            else if(sign >= 0){
                move_right(id);
            }
    }
}

double diffAngle(double angle1, double angle2){
    double res = angle1 - angle2;
    return res;
}

void stop(int id){
    vel[id].linear.x = 0;
    vel[id].angular.z = 0;
    vel_pub[id].publish(vel[id]);
    ros::spinOnce();
}

void move_forward(int id, double error_theta){
    vel[id].linear.x = SPEED_BITS;
    vel[id].angular.z = -GAIN_FWD*error_theta;
    vel_pub[id].publish(vel[id]);
    ros::spinOnce();
}

void move_back(int id){
    vel[id].linear.x = -SPEED_BITS;
    vel[id].angular.z = 0;
    vel_pub[id].publish(vel[id]);
    ros::spinOnce();
}

void move_left(int id){
    vel[id].linear.x = 0;
    vel[id].angular.z = SPEED_BITS;
    vel_pub[id].publish(vel[id]);
    ros::spinOnce();
}

void move_right(int id){
    vel[id].linear.x = 0;
    vel[id].angular.z = -SPEED_BITS;
    vel_pub[id].publish(vel[id]);
    ros::spinOnce();
}

void shoot(int id){
    shootandturbo[id].shoot = true;
    shootandturbo[id].turbo = false;
    shootandturbo_pub[id].publish(shootandturbo[id]);
    ros::spinOnce();
}

void turbo(int id){
    vel[id].angular.z = TURBO;
    vel_pub[id].publish(vel[id]);
    ros::spinOnce();
}

void followPath(vector<Node> vec){
    vector<Node>::iterator it = vec.begin();
    it = next(it);
    while(it != vec.end()){
        Node n = *it;
        GoToXY(0,gridXtoPixel(n.x),gridYtoPixel(n.y));
        if(state_gotoxy == END){
            it = next(it);
            end_gotoxy = false;
            state_gotoxy = ROTATE;
        }
    }
}

void create_map_astar(int space){
    QPointF topleft = QPointF(offset + borderOffset_x,offset);
    QPointF bottomleft = QPointF(offset + borderOffset_x,offset + projectedImageSizeY);
    QPointF topright = QPointF(offset + borderOffset_x + projectedImageSizeX,offset);
    QPointF bottomright = QPointF(offset + borderOffset_x + projectedImageSizeX,offset + projectedImageSizeY);

    for(int i=0;i<map_astar_size_x;i++){
        for(int j=0;j<map_astar_size_y;j++){
            map_astar[i][j] = true;
        }
    }

    for(int k=115;k<225;k++){           //Parede 1
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((200+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                    map_astar[(200+i-offset-70+space)/space - 1][(k+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=200;k<310;k++){          //Parede 2
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((k+i-offset-70+space) % space == 0 && (225+j-offset+space) % space == 0){
                    map_astar[(k+i-offset-70+space)/space - 1][(225+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=525;k<635;k++){           //Parede 3
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((200+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                    map_astar[(200+i-offset-70+space)/space - 1][(k+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=200;k<310;k++){           //Parede 4
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((k+i-offset-70+space) % space == 0 && (525+j-offset+space) % space == 0){
                    map_astar[(k+i-offset-70+space)/space - 1][(525+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=115;k<225;k++){           //Parede 5
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((844+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                    map_astar[(844+i-offset-70+space)/space - 1][(k+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=734;k<844;k++){           //Parede 6
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((k+i-offset-70+space) % space == 0 && (225+j-offset+space) % space == 0){
                    map_astar[(k+i-offset-70+space)/space - 1][(225+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=525;k<635;k++){           //Parede 7
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((844+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                    map_astar[(844+i-offset-70+space)/space - 1][(k+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=734;k<844;k++){           //Parede 8
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((k+i-offset-70+space) % space == 0 && (525+j-offset+space) % space == 0){
                    map_astar[(k+i-offset-70+space)/space - 1][(525+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=367;k<607;k++){           //Parede 9
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((k+i-offset-70+space) % space == 0 && (375+j-offset+space) % space == 0){
                    map_astar[(k+i-offset-70+space)/space - 1][(375+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=255;k<495;k++){           //Parede 10
        for(int i=-GAP;i<GAP+1;i++){
            for(int j=-GAP;j<GAP+1;j++){
                if((487+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                    map_astar[(487+i-offset-70+space)/space - 1][(k+j-offset+space)/space - 1] = false;
                }
            }
        }
    }

    for(int k=offset;k<projectedImageSizeY;k++){  //Border esquerda
        for(int i=0;i<GAP+1;i++){
            if((70+i+offset-offset-70+space) % space == 0 && (k-offset+space) % space == 0){
                map_astar[(70+i+offset-offset-70+space)/space - 1][(k-offset+space)/space - 1] = false;
            }
        }
    }

    for(int k=70+offset;k<projectedImageSizeX+offset;k++){  //Border cima
        for(int i=0;i<GAP+1;i++){
            if((k-offset-70+space) % space == 0 && (i+offset-offset+space) % space == 0){
                map_astar[(k-offset-70+space)/space - 1][(i+offset-offset+space)/space - 1] = false;
            }
        }
    }

    for(int k=70+offset;k<projectedImageSizeX+offset;k++){  //Border baixo
        for(int i=0;i<GAP+1;i++){
            if((k-offset-70+space) % space == 0 && (i+offset+projectedImageSizeY-GAP-offset+space) % space == 0){
                map_astar[(k-offset-70+space)/space - 1][(i+offset+projectedImageSizeY-GAP-offset+space)/space - 1] = false;
            }
        }
    }

    for(int k=offset;k<projectedImageSizeY;k++){             //Border direita
        for(int i=0;i<GAP+1;i++){
            if((i+offset+projectedImageSizeX-GAP-offset-70+space) % space == 0 && (k-offset+space) % space == 0){
                map_astar[(i+offset+projectedImageSizeX-GAP-offset-70+space)/space - 1][(k-offset+space)/space - 1] = false;
            }
        }
    }

    for(int i=0;i<map_astar_size_y;i++){
        for(int j=0;j<map_astar_size_x;j++){
        }
    }
}

static bool isValid(int x, int y){
    if(x>0 && y>0 && x<map_astar_size_x && y<map_astar_size_y){
        if(map_astar[x][y])
            return true;
        return false;
    }
    return false;
}

static bool isDestination(int x, int y, Node dest){
    if (x == dest.x && y == dest.y) {
        return true;
    }
    return false;
}

static double calculateH(int x, int y, Node dest){
    double H = abs(x-dest.x) + abs(y-dest.y);
    return H;
}

static vector<Node> aStar(Node player, Node dest){
    vector<Node> empty;
            if (isValid(dest.x, dest.y) == false) {
                return empty;
                //Destination is invalid
            }
            if (isDestination(player.x, player.y, dest)) {
                empty.push_back(dest);
                return empty;
            }
            bool closedList[(projectedImageSizeX-borderOffset_x)/SPACE + 1][(projectedImageSizeY)/SPACE + 1];

            //Initialize whole map
            //Node allMap[50][25];
            array<array < Node, (projectedImageSizeY)/SPACE + 1>, (projectedImageSizeX-borderOffset_x)/SPACE + 1> allMap;
            for (int x = 0; x < map_astar_size_x; x++) {
                for (int y = 0; y < map_astar_size_y; y++) {
                    allMap[x][y].fCost = FLT_MAX;
                    allMap[x][y].gCost = FLT_MAX;
                    allMap[x][y].hCost = FLT_MAX;
                    allMap[x][y].parentX = -1;
                    allMap[x][y].parentY = -1;
                    allMap[x][y].x = x;
                    allMap[x][y].y = y;

                    closedList[x][y] = false;
                }
            }

            //Initialize our starting list
            int x = player.x;
            int y = player.y;
            allMap[x][y].fCost = 0.0;
            allMap[x][y].gCost = 0.0;
            allMap[x][y].hCost = 0.0;
            allMap[x][y].parentX = x;
            allMap[x][y].parentY = y;

            vector<Node> openList;
            openList.emplace_back(allMap[x][y]);
            bool destinationFound = false;

            while (!openList.empty()&&openList.size()<map_astar_size_x*map_astar_size_y) {
                Node node;
                do {
                    float temp = FLT_MAX;
                    vector<Node>::iterator itNode;
                    for (vector<Node>::iterator it = openList.begin();
                        it != openList.end(); it = next(it)) {
                        Node n = *it;
                        if (n.fCost < temp) {
                            temp = n.fCost;
                            itNode = it;
                        }
                    }
                    node = *itNode;
                    openList.erase(itNode);
                } while (isValid(node.x, node.y) == false);

                x = node.x;
                y = node.y;
                closedList[x][y] = true;

                //For each neighbour starting from North-West to South-East
                for (int newX = -1; newX <= 1; newX++) {
                    for (int newY = -1; newY <= 1; newY++) {
                        double gNew, hNew, fNew;
                        if (isValid(x + newX, y + newY)) {
                            if (isDestination(x + newX, y + newY, dest))
                            {
                                //Destination found - make path
                                allMap[x + newX][y + newY].parentX = x;
                                allMap[x + newX][y + newY].parentY = y;
                                destinationFound = true;
                                return makePath(allMap, dest);
                            }
                            else if (closedList[x + newX][y + newY] == false)
                            {
                                gNew = node.gCost + 1.0;
                                hNew = calculateH(x + newX, y + newY, dest);
                                fNew = gNew + hNew;
                                // Check if this path is better than the one already present
                                if (allMap[x + newX][y + newY].fCost == FLT_MAX ||
                                    allMap[x + newX][y + newY].fCost > fNew)
                                {
                                    // Update the details of this neighbour node
                                    allMap[x + newX][y + newY].fCost = fNew;
                                    allMap[x + newX][y + newY].gCost = gNew;
                                    allMap[x + newX][y + newY].hCost = hNew;
                                    allMap[x + newX][y + newY].parentX = x;
                                    allMap[x + newX][y + newY].parentY = y;
                                    openList.emplace_back(allMap[x + newX][y + newY]);
                                }
                            }
                        }
                    }
                }
                }
                if (destinationFound == false) {
                    return empty;
            }
}

static vector<Node> makePath(array<array<Node, (projectedImageSizeY)/SPACE + 1>, ((projectedImageSizeX-borderOffset_x)/SPACE + 1)> map, Node dest){
    try {
                //cout << "Found a path" << endl;
                int x = dest.x;
                int y = dest.y;
                stack<Node> path;
                vector<Node> usablePath;
                int tempX, tempY, temp2X, temp2Y;

                while (!(map[x][y].parentX == x && map[x][y].parentY == y)
                    && map[x][y].x != -1 && map[x][y].y != -1)
                {
                    if(!path.empty()){
                        if(doIntersect2(new QPointF(gridXtoPixel(map[x][y].x), gridYtoPixel(map[x][y].y)), new QPointF(gridXtoPixel(path.top().x),gridYtoPixel(path.top().y)))){
                              path.push(map[temp2X][temp2Y]);
                        }
                    }
                    else path.push(map[x][y]);
                    tempX = map[x][y].parentX;
                    tempY = map[x][y].parentY;
                    temp2X = x;
                    temp2Y = y;
                    x = tempX;
                    y = tempY;
                }
                path.push(map[x][y]);

                while (!path.empty()) {
                    Node top = path.top();
                    path.pop();
                    usablePath.emplace_back(top);
                }
                return usablePath;
                        }
                        catch(const exception& e){
                            //cout << e.what() << endl;
                        }
}

void printNodes(vector<Node> vec){
    int aux = 1;
    int map_aux[(projectedImageSizeX-borderOffset_x)/SPACE + 1][(projectedImageSizeY)/SPACE + 1] = {};
    for (std::vector<Node>::iterator it = vec.begin(); it != vec.end(); it=next(it)){
        Node n = *it;
        map_aux[n.x][n.y] = aux;
        aux++;
    }
    printf("\n");
    for(int i=0;i<map_astar_size_y;i++){
        for(int j=0;j<map_astar_size_x;j++){
            if(map_aux[j][i])
                printf("%d", map_aux[j][i]);
            else if(map_astar[j][i])
                printf(" ");
            else
                printf("O");
        }
        printf("\n");
    }
}

int gridXtoPixel(int num){
    return num*SPACE+70+offset;
}

int gridYtoPixel(int num){
    return num*SPACE+offset;
}

int pixelXtoGrid(int num){
    return (int)round((float)(num-70-offset)/(float)SPACE);
}

int pixelYtoGrid(int num){
    return (int)round((float)(num-offset)/(float)SPACE);
}

QPointF* convertToGrid(int x, int y){

    QPointF* p = new QPointF();
    float x1 = (float)(x-70-offset)/(float)SPACE;
    float y1 = (float)(y-offset)/(float)SPACE;
    float min_dist = 9999;
    float dist = 9999;
    int ax = (int)floor((float)(x-70-offset)/(float)SPACE);
    int ay = (int)floor((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p->setX(ax);
            p->setY(ay);
        }
    }
    ax = (int)ceil((float)(x-70-offset)/(float)SPACE);
    ay = (int)floor((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p->setX(ax);
            p->setY(ay);
        }
    }
    ax = (int)floor((float)(x-70-offset)/(float)SPACE);
    ay = (int)ceil((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p->setX(ax);
            p->setY(ay);
        }
    }
    ax = (int)ceil((float)(x-70-offset)/(float)SPACE);
    ay = (int)ceil((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p->setX(ax);
            p->setY(ay);
        }
    }
    return p;
}

Node convertToNodeGrid(int x, int y){

    Node p;
    float x1 = (float)(x-70-offset)/(float)SPACE;
    float y1 = (float)(y-offset)/(float)SPACE;
    float min_dist = 9999;
    float dist = 9999;
    int ax = (int)floor((float)(x-70-offset)/(float)SPACE);
    int ay = (int)floor((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p.x = ax;
            p.y = ay;
        }
    }
    ax = (int)ceil((float)(x-70-offset)/(float)SPACE);
    ay = (int)floor((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p.x = ax;
            p.y = ay;
        }
    }
    ax = (int)floor((float)(x-70-offset)/(float)SPACE);
    ay = (int)ceil((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p.x = ax;
            p.y = ay;
        }
    }
    ax = (int)ceil((float)(x-70-offset)/(float)SPACE);
    ay = (int)ceil((float)(y-offset)/(float)SPACE);
    if (map_astar[ax][ay]){
        dist = abs(x1-ax)+abs(y1-ay);
        if(dist < min_dist){
            min_dist = dist;
            p.x = ax;
            p.y = ay;
        }
    }
    return p;
}

void moveToRandomPosition(int id){
    int x = rand() % map_astar_size_x;
    int y = rand() % map_astar_size_y;
    if(isValid(x,y) && !flag){
        Node player;
        player.x = pixelXtoGrid(robot[id].x);
        player.y = pixelYtoGrid(robot[id].y);
        Node dest;
        dest.x = x;
        dest.y = y;
        std::vector<Node> node_path;
        node_path = aStar(player,dest);
        printNodes(node_path);
        costTimeOfPath(id,node_path);
        followPath(node_path);
        rotateToEnemy(id,robot[1].x,robot[1].y);
        bool b = doIntersect(new QPointF(robot[id].x,robot[id].y), new QPointF(robot[1].x,robot[1].y));
        shoot(id);
        flag = true;
    }
}

bool checkTrajectoryCollision(int p0_x, int p0_y, int p1_x, int p1_y,
                              int p2_x, int p2_y, int p3_x, int p3_y){

        int s1_x, s1_y, s2_x, s2_y;
        s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
        s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

        float s, t;
        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1){
            return true;
        }
        return false; // No collision
}

bool onSegment(QPointF *p, QPointF *q, QPointF *r)
{
    if (q->x() <= max(p->x(), r->x()) && q->x() >= min(p->x(), r->x()) &&
        q->y() <= max(p->y(), r->y()) && q->y() >= min(p->y(), r->y()))
       return true;

    return false;
}


int orientation(QPointF *p, QPointF *q, QPointF *r)
{
    int val = (q->y() - p->y()) * (r->x() - q->x()) -
              (q->x() - p->x()) * (r->y() - q->y());

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool doIntersect(QPointF *p1, QPointF *q1)
{
    int o1,o2,o3,o4,tempx,tempy;
    QPointF *p2, *q2;

    for(int i=0;i<w_array.wall.size();i++){

        for(int j=0;j<4;j++){

            switch (j){

            case 0:
                tempx = -wallWidth/2;
                tempy = wallWidth/2;
                break;
            case 1:
                tempx = wallWidth/2;
                tempy = wallWidth/2;
                break;
            case 2:
                tempx = -wallWidth/2;
                tempy = -wallWidth/2;
                break;
            case 3:
                tempx = wallWidth/2;
                tempy = -wallWidth/2;
                break;
            }

            p2 = new QPointF(w_array.wall[i].x1+tempx,w_array.wall[i].y1+tempy);
            q2 = new QPointF(w_array.wall[i].x2+tempx,w_array.wall[i].y2+tempy);

            o1 = orientation(p1, q1, p2);
            o2 = orientation(p1, q1, q2);
            o3 = orientation(p2, q2, p1);
            o4 = orientation(p2, q2, q1);

            // General case
            if (o1 != o2 && o3 != o4)
                return true;


            if (o1 == 0 && onSegment(p1, p2, q1)) return true;

            if (o2 == 0 && onSegment(p1, q2, q1)) return true;

            if (o3 == 0 && onSegment(p2, p1, q2)) return true;

            if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        }
    }
    return false;
}

bool doIntersect2(QPointF *p1, QPointF *q1)
{
    int o1,o2,o3,o4,tempx,tempy;
    QPointF *p2, *q2;


    for(int i=0;i<w_array.wall.size();i++){

        for(int j=0;j<4;j++){

            switch (j){

            case 0:
                tempx = -GAP2;
                tempy = GAP2;
                break;
            case 1:
                tempx = GAP2;
                tempy = GAP2;
                break;
            case 2:
                tempx = -GAP2;
                tempy = -GAP2;
                break;
            case 3:
                tempx = GAP2;
                tempy = -GAP2;
                break;
            }

            p2 = new QPointF(w_array.wall[i].x1+tempx,w_array.wall[i].y1+tempy);
            q2 = new QPointF(w_array.wall[i].x2+tempx,w_array.wall[i].y2+tempy);

            o1 = orientation(p1, q1, p2);
            o2 = orientation(p1, q1, q2);
            o3 = orientation(p2, q2, p1);
            o4 = orientation(p2, q2, q1);

            if (o1 != o2 && o3 != o4)
                return true;

            if (o1 == 0 && onSegment(p1, p2, q1)) return true;

            if (o2 == 0 && onSegment(p1, q2, q1)) return true;

            if (o3 == 0 && onSegment(p2, p1, q2)) return true;

            if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        }
    }
    return false;
}

bool checkBehindWallCluster(QPointF *p1, QPointF *q1, int wall_cluster)
{
    int o1,o2,o3,o4,tempx,tempy;
    QPointF *p2, *q2;
    int cont = 0;

    for(int i=0;i<w_array.wall.size();i++){

        if(wall_cluster == floor(w_array.wall[i].id/2)){

            if(cont<2){

                cont++;

                for(int j=0;j<4;j++){

                    switch (j){

                    case 0:
                        tempx = -wallWidth/2;
                        tempy = wallWidth/2;
                        break;
                    case 1:
                        tempx = wallWidth/2;
                        tempy = wallWidth/2;
                        break;
                    case 2:
                        tempx = -wallWidth/2;
                        tempy = -wallWidth/2;
                        break;
                    case 3:
                        tempx = wallWidth/2;
                        tempy = -wallWidth/2;
                        break;
                    }

                    p2 = new QPointF(w_array.wall[i].x1+tempx,w_array.wall[i].y1+tempy);
                    q2 = new QPointF(w_array.wall[i].x2+tempx,w_array.wall[i].y2+tempy);

                    o1 = orientation(p1, q1, p2);
                    o2 = orientation(p1, q1, q2);
                    o3 = orientation(p2, q2, p1);
                    o4 = orientation(p2, q2, q1);

                    if (o1 != o2 && o3 != o4)
                        return true;

                    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

                    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

                    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

                    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
                }
            }
            else return false;
        }
    }
    return false;
}

float distanceTwoPoints(Node a, Node b){
    return sqrt((gridXtoPixel(a.x)-gridXtoPixel(b.x))*(gridXtoPixel(a.x)-gridXtoPixel(b.x))+(gridYtoPixel(a.y)-gridYtoPixel(b.y))*(gridYtoPixel(a.y)-gridYtoPixel(b.y)));
}

float distanceTwoPoints2(PlayerPosition a, PlayerPosition b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

float costTimeOfPath(int id, vector<Node> path){
    float linear_time = 0;
    float angular_time = 0;
    float angle1 = robot[id].angle;
    float angle2 = 0;
    float total = 0;
    for(int i=0; i<path.size()-1; i++){
        linear_time = (distanceTwoPoints(path[i],path[i+1])/100) / LINEAR_SPEED;
        angle2 = atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x) * 180 / PI;
        angular_time = ((abs(normDegrees180(diffAngle(angle2,angle1))) * PI) / 180) / ANGULAR_SPEED;
        angle1 = angle2;
        total = total + linear_time + angular_time;
    }
    return total;
}

float angleAfterMovement(vector<Node> path, PlayerPosition player){
    float angle1 = player.angle;
    float angle2 = 0;
    for(int i=0; i<path.size()-1; i++){
        angle2 = atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x) * 180 / PI;
        angle1 = angle2;
    }
    return angle2;
}

vector<Node> bestPointToAttack(int id_player, QPointF *p2){
    Node player;
    Node dest;
    QPointF* p1 = new QPointF();
    vector<Node> node_path, best_path;
    float cost = 999999;
    float min_cost = 999999;
    for(int k=0; k<map_astar_size_x;k++){
        for(int l=0; l<map_astar_size_y;l++){
            if(map_astar[k][l]){
                if(!doIntersect(new QPointF(gridXtoPixel(k),gridYtoPixel(l)), p2)){
                    p1 = convertToGrid(robot[id_player].x,robot[id_player].y);
                    player.x = p1->x();
                    player.y = p1->y();
                    dest.x = k;
                    dest.y = l;
                    node_path = aStar(player,dest);
                    cost = costTimeOfPath(id_player,node_path);
                    if(cost < min_cost){
                        min_cost = cost;
                        best_path = node_path;
                    }
                }
            }
        }
    }
    return best_path;
}

void moveToBestAttackPosition(int id_player, int id_enemy){
    followPath(bestPointToAttack(id_player, new QPointF(robot[id_enemy].x,robot[id_enemy].y)));
    if(!doIntersect(new QPointF(robot[id_player].x,robot[id_player].y), new QPointF(robot[id_enemy].x,robot[id_enemy].y))){
        rotateToEnemy(id_player,robot[id_enemy].x,robot[id_enemy].y);
        shoot(id);
    }
    stop(id);
}

void moveToBestDefensePosition(int id_player, int id_enemy){
    vector<Node> path;
    path = bestPointToDefend(id_player, id_enemy);
    printf("3\n");
    if(!path.empty()){
        followPath(path);
    }
    /*if(!doIntersect(new QPointF(robot[id_player].x,robot[id_player].y), new QPointF(robot[id_enemy].x,robot[id_enemy].y))){
        rotateToEnemy(id_player,robot[id_enemy].x,robot[id_enemy].y);
        shoot(id);
    }*/
    stop(id_player);
}

vector<Node> bestPointToDefend(int id_player, int id_enemy){
    //ver pontos fora do alcance inimigo
    //de esses pontos ver o ponto com menor custo de ida e maior custo para o inimigo
    Node player;
    Node dest;
    vector<Node> node_path, best_path, aux_path;
    float cost = 999999;
    float cost_enemy;
    float min_cost = 999999;
    int wall_cluster = -1;
    aux_path = bestPointToAttack(id_enemy, new QPointF(robot[id_player].x,robot[id_player].y));
    cost_enemy = costTimeOfPath(id_enemy,aux_path);
    for(int i=0; i<map_astar_size_x;i++){
        for(int j=0; j<map_astar_size_y;j++){
            if(map_astar[i][j]){
                wall_cluster = findBestWalls(id_player);
                if(checkBehindWallCluster(new QPointF(gridXtoPixel(i),gridYtoPixel(j)), new QPointF(gridXtoPixel(aux_path.back().x),gridYtoPixel(aux_path.back().y)), wall_cluster)){
                    dest.x = i;
                    dest.y = j;
                    player.x = pixelXtoGrid(robot[id_player].x);
                    player.y = pixelYtoGrid(robot[id_player].y);
                    node_path = aStar(player,dest);
                    cost = costTimeOfPath(id_player,node_path) - cost_enemy + HIST_COST_DEF;
                    if(cost < min_cost){
                        min_cost = cost;
                        best_path = node_path;
                    }
                }
            }
        }
    }
    printf("MIN COST DEF: %f\n", min_cost);
    if(min_cost > 0) return best_path;
    printf("1\n");
    vector<Node> empty;
    return empty;
}

int findBestWalls(int id){
    int dist = 99999;
    int dist_min = 99999;
    int wall_cluster = -1;
    for(int i=0;i<w_array.wall.size();i++){
        dist = abs(w_array.wall[i].x1 - robot[id].x) + abs(w_array.wall[i].y1 - robot[id].y);
        if(dist < dist_min){
            dist_min = dist;
            wall_cluster = floor(w_array.wall[i].id/2);
        }
        dist = abs(w_array.wall[i].x2 - robot[id].x) + abs(w_array.wall[i].y2 - robot[id].y);
        if(dist < dist_min){
            dist_min = dist;
            wall_cluster = floor(w_array.wall[i].id/2);
        }
    }
    //printf("Best wall cluster: %d\n", wall_cluster);
    return wall_cluster;
}

float negaMax(int depth, int player, Node player_pos, Node enemy_pos, Node pos_init){
    if (depth == 0)
        return sign[player]*evaluation(player, player_pos, enemy_pos, pos_init);
    float min = 99999;
    pos_init.x = player_pos.x;
    pos_init.y = player_pos.y;
    for (int i=0; i<map_astar_size_x;i++){
        for(int j=0; j<map_astar_size_y;j++){
            if (map_astar[i][j]){
                player_pos.x = i;
                player_pos.y = j;
                float cost = -negaMax(depth-1, 1-player, enemy_pos, player_pos, pos_init);
                if( cost < min ){
                    min = cost;
                    if(depth == DEPTH_NEGAMAX)
                        aux_node_path_min = aStar(pos_init,player_pos);
                }
            }
        }
    }
    printf("DEPTH: %d;    MIN: %f\n", depth, min);
    return min;
}

float negaMax2(int depth, int player, Node player_pos, Node enemy_pos, Node pos_init, float cost){
    if (depth == 0)
        return sign[player]*cost;
    float min = 99999;
    pos_init.x = player_pos.x;
    pos_init.y = player_pos.y;
    for (int i=0; i<map_astar_size_x;i++){
        for(int j=0; j<map_astar_size_y;j++){
            if (map_astar[i][j]){
                player_pos.x = i;
                player_pos.y = j;
                float aux_cost = cost;
                cost = -cost + evaluation2(player, player_pos, enemy_pos, pos_init);
                cost = -negaMax2(depth-1, 1-player, enemy_pos, player_pos, pos_init, cost);
                if( cost < min ){
                    min = cost;
                    if(depth == DEPTH_NEGAMAX)
                        aux_node_path_min = aStar(pos_init,player_pos);
                }
                //else{
                    cost = aux_cost;
                //}
            }
        }
    }
    printf("DEPTH: %d;    MIN: %f\n", depth, min);
    return min;
}

float negaMax3(int depth, int player, Node player_pos, Node enemy_pos){
    if (depth == 0)
        return sign[player]*evaluation3(player, player_pos, enemy_pos);
    float max = -99999;
    Node pos_init;
    pos_init.x = player_pos.x;
    pos_init.y = player_pos.y;
    for (int i=player_pos.x-max_cells; i<player_pos.x+max_cells+1;i++){
        for(int j=player_pos.y-max_cells; j<player_pos.y+max_cells+1;j++){
            //printf("PLAYER: %d, %d\n", player_pos.x,player_pos.y);
            //printf("I: %d  J: %d\n", i, j);
            if((i<map_astar_size_x) && (j<map_astar_size_y)){
                if (map_astar[i][j]){
                    player_pos.x = i;
                    player_pos.y = j;
                    float score = -negaMax3(depth-1, 1-player, enemy_pos, player_pos);
                    if( score > max ){
                        max = score;
                        if(depth == DEPTH_NEGAMAX)
                            aux_node_path_min = aStar(pos_init,player_pos);
                    }
                    player_pos.x = pos_init.x;
                    player_pos.y = pos_init.y;
                }
            }
        }
    }
    if(depth == DEPTH_NEGAMAX)
        printf("DEPTH: %d;    MAX: %f\n", depth, max);
    return max;
}

vector<w_id_inter> wallsBetweenPlayers(Node player_pos, Node enemy_pos){
    vector<w_id_inter> wall_ids;
    pdd intersection;
    for(int i=0;i<w_array.wall.size();i++){
        intersection = lineLineIntersection(make_pair(gridXtoPixel(player_pos.x),gridYtoPixel(player_pos.y)),make_pair(gridXtoPixel(enemy_pos.x),gridYtoPixel(enemy_pos.y)),make_pair(w_array.wall[i].x1, w_array.wall[i].y1),make_pair(w_array.wall[i].x2,w_array.wall[i].y2));
        if(intersection.first != FLT_MAX)
            wall_ids.push_back(make_pair(i,intersection));
    }
    return wall_ids;
}

vector<w_id_inter> wallsBetweenPlayers2(PlayerPosition player_pos, PlayerPosition enemy_pos){
    vector<w_id_inter> wall_ids;
    pdd intersection;
    for(int i=0;i<w_array.wall.size();i++){
        intersection = lineLineIntersection(make_pair(player_pos.x,player_pos.y),make_pair(enemy_pos.x,enemy_pos.y),make_pair(w_array.wall[i].x1, w_array.wall[i].y1),make_pair(w_array.wall[i].x2,w_array.wall[i].y2));
        if(intersection.first != FLT_MAX)
            wall_ids.push_back(make_pair(i,intersection));
    }
    //printf("walls: %d\n", wall_ids.size());
    return wall_ids;
}

float evaluation3(int player, Node player_pos, Node enemy_pos){
    vector<w_id_inter> walls;
    float dist_center = 0;
    float dist_vertice = 0;
    walls = wallsBetweenPlayers(player_pos,enemy_pos);
    if(!walls.empty()){
        for(int i=0;i<walls.size();i++){
            dist_center = dist_center + distToWallCenter(walls[i].second,walls[i].first);
            dist_vertice = dist_vertice + distToWallVertice(walls[i].second,walls[i].first,player);
        }
        return dist_center*-1*winning(player) + dist_vertice*winning(player);
    }
    else{
        return (500-distanceTwoPoints(player_pos,enemy_pos))*-1*winning(player);
    }
}

float negaMax4(int depth, int player, PlayerPosition player_pos, PlayerPosition enemy_pos){
    if (depth == 0)
        return evaluation4(player, player_pos, enemy_pos);
    float max = -99999;
    PlayerPosition pos_init;
    vector<Node> path;
    Node pos_init_node;
    Node player_pos_node;
    pos_init = player_pos;
    pos_init_node = convertToNodeGrid(pos_init.x,pos_init.y);
    player_pos.angle = atan2(enemy_pos.y-player_pos.y,enemy_pos.x-player_pos.x) * 180 / PI;
    float score = -negaMax4(depth-1, 1-player, enemy_pos, player_pos);
    if( score > max ){
        max = score;
        if(depth == DEPTH_NEGAMAX){
            play = ROTATE_TO_ENEMY;
        }
    }
    player_pos.angle = pos_init.angle;
    for (int i=pos_init_node.x-max_cells; i<pos_init_node.x+max_cells+1;i++){
        for(int j=pos_init_node.y-max_cells; j<pos_init_node.y+max_cells+1;j++){
            if((i<map_astar_size_x) && (j<map_astar_size_y) && (i>=0) && (j>=0)){
                if (map_astar[i][j]){
                    player_pos_node.x = i;
                    player_pos_node.y = j;
                    path = aStar(pos_init_node,player_pos_node);
                    player_pos.x = gridXtoPixel(i);
                    player_pos.y = gridYtoPixel(j);
                    player_pos.angle = angleAfterMovement(path,player_pos);
                    float score = -negaMax4(depth-1, 1-player, enemy_pos, player_pos);
                    if( score > max ){
                        max = score;
                        if(depth == DEPTH_NEGAMAX){
                            aux_node_path_min = aStar(pos_init_node,player_pos_node);
                            play = MOVE;
                        }
                    }
                    player_pos = pos_init;
                }
            }
        }
    }
    if(depth == DEPTH_NEGAMAX)
    return max;
}

bool obstacle(vector<w_id_inter> walls){
    if(walls.empty()) return 0;
    else return 1;
}

int iswinning(int id){
    //printf("class: %d\n", robot[id].classification);
    if(robot[id].classification == 1) return 1;
    return 0;
}

float evaluation4(int player, PlayerPosition player_pos, PlayerPosition enemy_pos){
    vector<w_id_inter> walls;
    float dist_center = 0;
    float dist_vertice = 0;
    float time_relative_to_bullet = 0;
    float time_players_rotation = 0;
    float dist_to_vertices = 0;
    float time_hidden_advantage = 0;
    float time_rotate_to_enemy_and_enemy_move = 0;
    walls = wallsBetweenPlayers2(player_pos,enemy_pos);
    if(walls.empty()){
        time_relative_to_bullet = (distanceTwoPoints2(player_pos,enemy_pos)/(bullet_speed/0.05))-((pi_radius/(914/1.608))/LINEAR_SPEED);
        //printf("TIME_RELATIVE_TO_BULLET:               %f\n", time_relative_to_bullet);
        time_players_rotation = (abs((normDegrees180(diffAngle((atan2(enemy_pos.y-player_pos.y,enemy_pos.x-player_pos.x) * 180 / PI),player_pos.angle))) * PI / 180)/ANGULAR_SPEED) - (abs((normDegrees180(diffAngle((atan2(player_pos.y-enemy_pos.y,player_pos.x-enemy_pos.x) * 180 / PI),enemy_pos.angle))) * PI / 180)/ANGULAR_SPEED);
        //printf("TIME_PLAYERS_ROTATION:                 %f\n", time_players_rotation);
        //return max(time_players_rotation,winning(player)*1*time_relative_to_bullet);
        //return time_players_rotation + winning(player)*1*time_relative_to_bullet;
        //return winning(player)*-1*time_relative_to_bullet;
    }
    else{
        for(int i=0;i<walls.size();i++){
            dist_center = dist_center + distToWallCenter(walls[i].second,walls[i].first);
            if(winning(player)==1){
                dist_vertice = dist_vertice + distToWallVertice2(walls[i].second,walls[i].first,enemy_pos);
                if(walls.size()<2)
                    dist_to_vertices = dist_to_vertices + distanceTwoPoints2(enemy_pos,closestVertice(enemy_pos,walls[i].first));
                else
                    dist_to_vertices = 99999;
            }
            else{
                dist_vertice = dist_vertice + distToWallVertice2(walls[i].second,walls[i].first,player_pos);
                if(walls.size()<2)
                    dist_to_vertices = dist_to_vertices + distanceTwoPoints2(player_pos,closestVertice(player_pos,walls[i].first));
                else
                    dist_to_vertices = 99999;
            }
        }
        time_hidden_advantage = (-1*winning(player)*((dist_center - dist_vertice))/(914/1.608))/LINEAR_SPEED;
        //printf("TIME_HIDDEN_ADVANTAGE:                  %f\n", time_hidden_advantage);
        if(winning(player) == 1){
            time_rotate_to_enemy_and_enemy_move = (abs((normDegrees180(diffAngle((atan2(enemy_pos.y-player_pos.y,enemy_pos.x-player_pos.x) * 180 / PI),player_pos.angle)))*PI/180)/ANGULAR_SPEED) - (dist_to_vertices/(914/1.608))/LINEAR_SPEED;
            //printf("TIME_ROTATE_TO_ENEMY_MOVE:          %f\n", time_rotate_to_enemy_and_enemy_move);
            //return max(time_hidden_advantage, time_rotate_to_enemy_and_enemy_move);
        }
        else{
            time_rotate_to_enemy_and_enemy_move = (abs((normDegrees180(diffAngle((atan2(enemy_pos.y-player_pos.y,enemy_pos.x-player_pos.x) * 180 / PI),enemy_pos.angle)))*PI/180)/ANGULAR_SPEED) - (dist_to_vertices/(914/1.608))/LINEAR_SPEED;
            //printf("TIME_ROTATE_TO_ENEMY_MOVE:          %f\n", time_rotate_to_enemy_and_enemy_move);
            //return time_rotate_to_enemy_and_enemy_move;
        }
        //return time_hidden_advantage+(-1*winning(player)*time_rotate_to_enemy_and_enemy_move);
        //return time_hidden_advantage;
    }
    return iswinning(player)*(1-obstacle(walls))*(winning(player)*time_relative_to_bullet) + iswinning(player)*obstacle(walls)*time_hidden_advantage + (1-iswinning(player))*(1-obstacle(walls))*(winning(player)*time_relative_to_bullet) + (1-iswinning(player))*obstacle(walls)*(walls.size()*time_hidden_advantage);
}

float evaluation(int player, Node player_pos, Node enemy_pos, Node pos_init){
    if (winning(1-player)){
    //if(true){
        if(doIntersect(new QPointF(gridXtoPixel(player_pos.x),gridYtoPixel(player_pos.y)), new QPointF(gridXtoPixel(enemy_pos.x),gridYtoPixel(enemy_pos.y)))){
            //if(player == 0){
                //aux_node_path.clear();
                aux_node_path = aStar(pos_init,enemy_pos);
            //}
            return sign[1-player]*costTimeOfPath(1-player,aux_node_path);
        }
        return sign[1-player]*9999;
    }
    else{
        if(!doIntersect(new QPointF(gridXtoPixel(player_pos.x),gridYtoPixel(player_pos.y)), new QPointF(gridXtoPixel(enemy_pos.x),gridYtoPixel(enemy_pos.y)))){
           //if(player == 0){
                //aux_node_path.clear();
                aux_node_path = aStar(pos_init,enemy_pos);
            //}
            return sign[1-player]*costTimeOfPath(1-player,aux_node_path);
        }
        return sign[1-player]*9999;
    }
}

float evaluation2(int player, Node player_pos, Node enemy_pos, Node pos_init){
    aux_node_path = aStar(pos_init,player_pos);
    float cost = costTimeOfPath(player,aux_node_path);
    if (winning(player)){
        if(doIntersect(new QPointF(gridXtoPixel(player_pos.x),gridYtoPixel(player_pos.y)), new QPointF(gridXtoPixel(enemy_pos.x),gridYtoPixel(enemy_pos.y)))){
            return cost;
        }
        return cost+9999;
    }
    else{
        if(!doIntersect(new QPointF(gridXtoPixel(player_pos.x),gridYtoPixel(player_pos.y)), new QPointF(gridXtoPixel(enemy_pos.x),gridYtoPixel(enemy_pos.y)))){
            return cost;
        }
        return cost+9999;
    }
}

int winning(int id){
    if(robot[id].classification == 1) return 1;
    return -1;
}

int distToWallCenter(pdd p, int wall_id){
    if(w_array.wall[wall_id].vertical) return abs(p.second - w_array.wall[wall_id].yc);
    else return abs(p.first - w_array.wall[wall_id].xc);
}

PlayerPosition closestVertice(PlayerPosition p, int wall_id){
    if(abs(p.x - w_array.wall[wall_id].x1) + abs(p.y - w_array.wall[wall_id].y1) <= abs(p.x - w_array.wall[wall_id].x2) + abs(p.y - w_array.wall[wall_id].y2)){
        PlayerPosition vertice;
        vertice.x = w_array.wall[wall_id].x1;
        vertice.y = w_array.wall[wall_id].y1;
        return vertice;
    }
    else{
        PlayerPosition vertice;
        vertice.x = w_array.wall[wall_id].x2;
        vertice.y = w_array.wall[wall_id].y2;
        return vertice;
    }
}

int distToWallVertice(pdd p, int wall_id, int player_id){
    if(abs(robot[player_id].x - w_array.wall[wall_id].x1) + abs(robot[player_id].y - w_array.wall[wall_id].y1) <= abs(robot[player_id].x - w_array.wall[wall_id].x2) + abs(robot[player_id].y - w_array.wall[wall_id].y2))
        return abs(p.second - w_array.wall[wall_id].y1)+abs(p.first - w_array.wall[wall_id].x1);
    else return abs(p.second - w_array.wall[wall_id].y2)+abs(p.first - w_array.wall[wall_id].x2);
}

int distToWallVertice2(pdd p, int wall_id, PlayerPosition player){
    if(abs(player.x - w_array.wall[wall_id].x1) + abs(player.y - w_array.wall[wall_id].y1) <= abs(player.x - w_array.wall[wall_id].x2) + abs(player.y - w_array.wall[wall_id].y2))
        return abs(p.second - w_array.wall[wall_id].y1)+abs(p.first - w_array.wall[wall_id].x1);
    else return abs(p.second - w_array.wall[wall_id].y2)+abs(p.first - w_array.wall[wall_id].x2);
}

pdd lineLineIntersection(pdd A, pdd B, pdd C, pdd D){
    // Line AB represented as a1x + b1y = c1
    double a1 = B.second - A.second;
    double b1 = A.first - B.first;
    double c1 = a1*(A.first) + b1*(A.second);

    // Line CD represented as a2x + b2y = c2
    double a2 = D.second - C.second;
    double b2 = C.first - D.first;
    double c2 = a2*(C.first)+ b2*(C.second);

    double determinant = a1*b2 - a2*b1;

    if (determinant == 0)
    {
        // The lines are parallel. This is simplified
        // by returning a pair of FLT_MAX
        return make_pair(FLT_MAX, FLT_MAX);
    }
    else
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        if((min(A.first,B.first)<=x)&&(x<=max(A.first,B.first))&&(min(A.second,B.second)<=y)&&(y<=max(A.second,B.second))   &&   (min(C.first,D.first)<=x)&&(x<=max(C.first,D.first))&&(min(C.second,D.second)<=y)&&(y<=max(C.second,D.second)))
            return make_pair(x, y);
        else return make_pair(FLT_MAX, FLT_MAX);
    }
}

void stateMachine(int id_player, int id_enemy){

    Node player_node = convertToNodeGrid(robot[id_player].x,robot[id_player].y);
    Node enemy_node = convertToNodeGrid(robot[id_enemy].x,robot[id_enemy].y);
    Node dest;
    PlayerPosition player_pos;
    PlayerPosition enemy_pos;
    player_pos.x = robot[0].x;
    player_pos.y = robot[0].y;
    player_pos.angle = robot[0].angle;
    enemy_pos.x = robot[1].x;
    enemy_pos.y = robot[1].y;
    enemy_pos.angle = robot[1].angle;

    vector<w_id_inter> walls;
    walls = wallsBetweenPlayers2(player_pos,enemy_pos);

    int macrostate = iswinning(id_player);
    int state = -1;

    if(walls.empty()) state = VISIBLE;
    else state = HIDDEN;

    switch (macrostate){

    case WINNING:
    {

        switch(state){

            case HIDDEN:
            {
            int min = 9999;
            for (int i=player_node.x-max_cells; i<player_node.x+max_cells+1;i++){
                for(int j=player_node.y-max_cells; j<player_node.y+max_cells+1;j++){
                    if((i<map_astar_size_x) && (j<map_astar_size_y) && (i>=0) && (j>=0)){
                        if (map_astar[i][j]){
                            float dist_center = -1;
                            Node temp;
                            temp.x = i;
                            temp.y = j;
                            vector<w_id_inter> ws = wallsBetweenPlayers(temp, enemy_node);
                            for(int k=0;k<ws.size();k++){
                                if(dist_center == -1)
                                    dist_center = distToWallCenter(ws[k].second,ws[k].first);
                                else
                                    dist_center = dist_center + distToWallCenter(ws[k].second,ws[k].first);
                            }
                            if((dist_center<min)&&(dist_center>=0)){
                                dest.x = i;
                                dest.y = j;
                                min = dist_center;
                            }
                        }
                    }
                }
            }
            followPath(aStar(player_node,dest));
            break;
            }
            case VISIBLE:
            {
            float max = distanceTwoPoints(player_node, enemy_node);
            for (int i=player_node.x-max_cells; i<player_node.x+max_cells+1;i++){
                for(int j=player_node.y-max_cells; j<player_node.y+max_cells+1;j++){
                    if((i<map_astar_size_x) && (j<map_astar_size_y) && (i>=0) && (j>=0)){
                        if (map_astar[i][j]){
                            Node temp;
                            temp.x = i;
                            temp.y = j;
                            vector<w_id_inter> ws = wallsBetweenPlayers(temp, enemy_node);
                            if(!ws.empty()){
                                dest.x = i;
                                dest.y = j;
                                goto END_LOOP;
                            }
                            else{
                                float dist = distanceTwoPoints(temp, enemy_node);
                                if(dist > max){
                                    dest.x = i;
                                    dest.y = j;
                                    max = dist;
                                }
                            }
                        }
                    }
                }
            }
            END_LOOP:
            followPath(aStar(player_node,dest));
            break;
            }
        }
        break;
    }
    case LOSING:
    {

        switch(state){

            case HIDDEN:
            {
            int max = -9999;
            int min_dist = 9999;
            for (int i=player_node.x-max_cells; i<player_node.x+max_cells+1;i++){
                for(int j=player_node.y-max_cells; j<player_node.y+max_cells+1;j++){
                    if((i<map_astar_size_x) && (j<map_astar_size_y) && (i>=0) && (j>=0)){
                        if (map_astar[i][j]){
                            float dist_center = -1;
                            Node temp;
                            temp.x = i;
                            temp.y = j;
                            vector<w_id_inter> ws = wallsBetweenPlayers(temp, enemy_node);
                            for(int k=0;k<ws.size();k++){
                                dist_center = dist_center + distToWallCenter(ws[k].second,ws[k].first);

                            }
                            //printf("dist center: %f\n", dist_center);
                            if(dist_center == -1 && (abs(i-player_node.x)+abs(j-player_node.y)) < min_dist){
                                dest.x = i;
                                dest.y = j;
                            }
                            else if(dist_center>max){
                                dest.x = i;
                                dest.y = j;
                                max = dist_center;
                            }
                        }
                    }
                }
            }
            followPath(aStar(player_node,dest));
            break;
            }
            case VISIBLE:
            {
            float min = distanceTwoPoints(player_node, enemy_node);
            rotateToEnemy(0,enemy_pos.x,enemy_pos.y);
            if(!doIntersect2(new QPointF(robot[0].x,robot[0].y), new QPointF(robot[1].x,robot[1].y)));
                move_forward(0,0);
            break;
            }
        }
        break;
    }
    }
    if(!doIntersect(new QPointF(robot[0].x,robot[0].y), new QPointF(robot[1].x,robot[1].y))){
        if(abs(normDegrees180(diffAngle((atan2(enemy_pos.y-player_pos.y,enemy_pos.x-player_pos.x) * 180 / PI),player_pos.angle))) < MAX_ETF)
            shoot(0);
    }
}

void decisionMaking(){
    PlayerPosition player;
    PlayerPosition enemy;
    player.x = robot[0].x;
    player.y = robot[0].y;
    player.angle = robot[0].angle;
    enemy.x = robot[1].x;
    enemy.y = robot[1].y;
    enemy.angle = robot[1].angle;
    negaMax4(DEPTH_NEGAMAX,0,player,enemy);
    if(play == MOVE){
        //printNodes(aux_node_path_min);
        followPath(aux_node_path_min);
    }
    else if(play == ROTATE_TO_ENEMY){
        rotateToEnemy(0,robot[1].x,robot[1].y);
        stop(0);
    }
    if(!doIntersect(new QPointF(robot[0].x,robot[0].y), new QPointF(robot[1].x,robot[1].y))){
        if(abs(normDegrees180(diffAngle((atan2(enemy.y-player.y,enemy.x-player.x) * 180 / PI),player.angle))) < MAX_ETF)
            shoot(0);
    }
}

int main(int argc, char **argv){

        ros::init(argc, argv, "ai");
        ros::NodeHandle nh;

        vel_pub[0] = nh.advertise<geometry_msgs::Twist>("/vel1", 1);
        vel_pub[1] = nh.advertise<geometry_msgs::Twist>("/vel2", 1);
        vel_pub[2] = nh.advertise<geometry_msgs::Twist>("/vel3", 1);
        vel_pub[3] = nh.advertise<geometry_msgs::Twist>("/vel4", 1);
        uistate_sub = nh.subscribe("/ui_state", 1000, &uiStateReceived);
        robot_sub = nh.subscribe("/robots_description", 1000, &robotDescriptionReceived);
        wall_sub = nh.subscribe("/wall_info", 1000, &wallInfoReceived);
        shootandturbo_pub[0] = nh.advertise<game_engine::ShootAndTurbo>("/robot1/shootandturbo", 1);
        shootandturbo_pub[1] = nh.advertise<game_engine::ShootAndTurbo>("/robot2/shootandturbo", 1);
        shootandturbo_pub[2] = nh.advertise<game_engine::ShootAndTurbo>("/robot3/shootandturbo", 1);
        shootandturbo_pub[3] = nh.advertise<game_engine::ShootAndTurbo>("/robot4/shootandturbo", 1);

        srand(time(NULL));
        create_map_astar(SPACE);
        max_cells = floor(float(PERIOD)/float(COST1CELL));

        while(1){
            if(uistate.gameStart && uistate.aiGame && (uistate.seconds>0)){
                decisionMaking();
                //stateMachine(0,1);
            }
            ros::spinOnce();
        }
}
