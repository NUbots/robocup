#ifndef VISIONCOMPARITOR_H
#define VISIONCOMPARITOR_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "Infrastructure/NUImage/NUImage.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

namespace Ui {
class VisionComparitor;
}

class VisionComparitor : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit VisionComparitor(QWidget *parent = 0);
    ~VisionComparitor();

    void run(std::string image_name, std::string sensor_name, std::string lut_name, std::string config0, std::string config1);

private:
    void display(const QImage& img0, const QImage& img1, const QImage& classifiedImg);

private slots:
    void halt() {m_halted=true;}
    void next() {m_next=true;}
    void prev() {m_prev=true;}

private:
    Ui::VisionComparitor *ui;   //! @var User interface pointer

    pair<QGraphicsScene,QGraphicsScene> m_scenes;               //! @var The two scene
    QGraphicsScene m_class_scene;
    pair<QGraphicsPixmapItem,QGraphicsPixmapItem> m_pixmaps;    //! @var The two pixmaps
    QGraphicsPixmapItem m_class_pixmap;

    unsigned int m_frame_no;    //! @var The current frame index
    std::vector<pair<NUImage, NUSensorsData> > m_frames;   //! @var The frames read in from file
    bool m_halted,              //! @var Flag for user selecting to exit
         m_next,                //! @var Flag for user selecting to go to next frame
         m_prev;                //! @var Flag for user selecting to go to prev frame
};

#endif // VISIONCOMPARITOR_H
