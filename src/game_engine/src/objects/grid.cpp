#include "../../include/objects/grid.hpp"

using namespace std;

grid::grid(QPointF topleft, QPointF topright, QPointF bottomleft, QPointF bottomright, int space){
    this->topleft = topleft;
    this->topright = topright;
    this->bottomleft = bottomleft;
    this->bottomright = bottomright;
    this->space = space;
    this->mode = MODE;
}

QRectF grid::boundingRect() const {
  return QRectF(0, 0, 1, 1);
}

QPainterPath grid::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void grid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
        QColor color1("grey");
        QPen pen1(color1,1);
        painter->setPen(pen1);
        //int i =0;
        for(int i=0; i<=topright.x()-topleft.x(); i=i+space){
            point.setY(topleft.y());
            point.setX(topleft.x()+i);
            line.setP1(point);
            point.setY(bottomleft.y());
            point.setX(bottomleft.x()+i);
            line.setP2(point);
            painter->drawLine(line);
        }
        for(int j=0; j<=bottomleft.y()-topleft.y(); j=j+space){
            point.setY(topleft.y()+j);
            point.setX(topleft.x());
            line.setP1(point);
            point.setY(topright.y()+j);
            point.setX(topright.x());
            line.setP2(point);
            painter->drawLine(line);
        }

    if (mode){

        QColor color2("red");
        QColor color3("blue");
        QPen pen2(color2,1);
        QPen pen3(color3,4);
        painter->setPen(pen2);
        QPointF p;

        for(int k=115;k<225;k++){           //Parede 1
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(200+i,k+j));
                    if((200+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(200+i,k+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=200;k<310;k++){          //Parede 2
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(k+i,225+j));
                    if((k+i-offset-70+space) % space == 0 && (225+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(k+i,225+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=525;k<635;k++){           //Parede 3
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(200+i,k+j));
                    if((200+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(200+i,k+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=200;k<310;k++){           //Parede 4
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(k+i,525+j));
                    if((k+i-offset-70+space) % space == 0 && (525+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(k+i,525+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=115;k<225;k++){           //Parede 5
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(844+i,k+j));
                    if((844+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(844+i,k+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=734;k<844;k++){           //Parede 6
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(k+i,225+j));
                    if((k+i-offset-70+space) % space == 0 && (225+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(k+i,225+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=525;k<635;k++){           //Parede 7
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(844+i,k+j));
                    if((844+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(844+i,k+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=734;k<844;k++){           //Parede 8
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(k+i,525+j));
                    if((k+i-offset-70+space) % space == 0 && (525+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(k+i,525+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=367;k<607;k++){           //Parede 9
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(k+i,375+j));
                    if((k+i-offset-70+space) % space == 0 && (375+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(k+i,375+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=255;k<495;k++){           //Parede 10
            for(int i=-38;i<39;i++){
                for(int j=-38;j<39;j++){
                    painter->drawPoint(QPointF(487+i,k+j));
                    if((487+i-offset-70+space) % space == 0 && (k+j-offset+space) % space == 0){
                        painter->setPen(pen3);
                        painter->drawPoint(QPointF(487+i,k+j));
                        painter->setPen(pen2);
                    }
                }
            }
        }

        for(int k=offset;k<projectedImageSizeY;k++){  //Border esquerda
            for(int i=0;i<39;i++){
                painter->drawPoint(QPointF(70+i+offset,k));
                if((70+i+offset-offset-70+space) % space == 0 && (k-offset+space) % space == 0){
                    painter->setPen(pen3);
                    painter->drawPoint(QPointF(70+i+offset,k));
                    painter->setPen(pen2);
                }
            }
        }

        for(int k=70+offset;k<projectedImageSizeX+offset;k++){  //Border cima
            for(int i=0;i<39;i++){
                painter->drawPoint(QPointF(k,i+offset));
                if((k-offset-70+space) % space == 0 && (i+offset-offset+space) % space == 0){
                    painter->setPen(pen3);
                    painter->drawPoint(QPointF(k,i+offset));
                    painter->setPen(pen2);
                }
            }
        }

        for(int k=70+offset;k<projectedImageSizeX+offset;k++){  //Border baixo
            for(int i=0;i<39;i++){
                painter->drawPoint(QPointF(k,i+offset+projectedImageSizeY-38));
                if((k-offset-70+space) % space == 0 && (i+offset+projectedImageSizeY-38-offset+space) % space == 0){
                    painter->setPen(pen3);
                    painter->drawPoint(QPointF(k,i+offset+projectedImageSizeY-38));
                    painter->setPen(pen2);
                }
            }
        }

        for(int k=offset;k<projectedImageSizeY;k++){
            for(int i=0;i<39;i++){
                painter->drawPoint(QPointF(i+offset+projectedImageSizeX-38,k));
                if((i+offset+projectedImageSizeX-38-offset-70+space) % space == 0 && (k-offset+space) % space == 0){
                    painter->setPen(pen3);
                    painter->drawPoint(QPointF(i+offset+projectedImageSizeX-38,k));
                    painter->setPen(pen2);
                }
            }
        }
    }
}

