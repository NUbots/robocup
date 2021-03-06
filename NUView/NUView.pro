QT += network \
      opengl

CONFIG += qwt

QMAKE_CXXFLAGS += -std=c++0x

macx { 
    # Mac Specific Includes
    QMAKE_LFLAGS += -F/System/Library/Frameworks/CoreFoundation.framework/
    LIBS += -framework CoreFoundation -lz
    DESTDIR = "../Build/NUView"
    OBJECTS_DIR = "../Build/NUView/.obj"
    MOC_DIR = "../Build/NUView/.moc"
    RCC_DIR = "../Build/NUView/.rcc"
    UI_DIR = "../Build/NUView/.ui"con
    #Macports include directory
    INCLUDEPATH += '/opt/local/include'
}
win32 { 
    INCLUDEPATH += 'C:/Program Files (x86)/boost/'
    INCLUDEPATH += 'C:/Program Files (x86)/boost/boost_1_42'
    INCLUDEPATH += 'C:/Qt/2010.02.1/qt/src/3rdparty/zlib'
    INCLUDEPATH += 'C:/Program Files (x86)/boost/boost_1_44'
    INCLUDEPATH += 'C:/Qt/2010.05/qt/src/3rdparty/zlib'
    INCLUDEPATH += 'C:/Program Files/boost/boost_1_44'
    INCLUDEPATH += 'C:\Program Files\Boost\boost_1_52_0'
    INCLUDEPATH += 'C:/Program Files/boost/'
    INCLUDEPATH += 'C:/Program Files/Bonjour SDK/Include/'
    LIBS += -lwsock32
    LIBS += -lpthread
    DEFINES += TARGET_OS_IS_WINDOWS
}
!macx{
    !win32{
        INCLUDEPATH += /usr/include/boost/
        LIBS += -ldns_sd -lGLU -lzmq -lprotobuf
    }
}

DEFINES += TARGET_IS_NUVIEW

# Opencv library
# INCLUDEPATH += "C:\Program Files\OpenCV\cv\include" "C:\Program Files\OpenCV\cvaux\include" "C:\Program Files\OpenCV\cxcore\include" "C:\Program Files\OpenCV\otherlibs\highgui"
# LIBS += -L"C:\Program Files\OpenCV\lib" -lcv -lcvaux -lhighgui -lcxcore
# Input
# HEADERS += "C:\Program Files\OpenCV\cv\include\cv.h" "C:\Program Files\OpenCV\otherlibs\highgui\highgui.h"
INCLUDEPATH += ../
INCLUDEPATH += ../NUview/
INCLUDEPATH += NUViewConfig/
HEADERS += ui_mainwindow.h \
    mainwindow.h \
    connectionwidget.h \
    ColorModelConversions.h \
    classificationwidget.h \
    ../Tools/FileFormats/NUbotImage.h \
    #../VisionOld/Vision.h \
    ../Tools/FileFormats/LUTTools.h \
    virtualnubot.h \
    ../Infrastructure/NUImage/BresenhamLine.h \
    ../Tools/Math/Vector2.h \
    ../Tools/Math/Line.h \
    ../Tools/Math/Circle.h \
    ../Kinematics/Horizon.h \
    openglmanager.h \
    GLDisplay.h \
    ../Infrastructure/NUImage/NUImage.h \
    ../Infrastructure/NUImage/ClassifiedImage.h \
    #../VisionOld/ClassifiedSection.h \
    #../VisionOld/ScanLine.h \
    #../VisionOld/TransitionSegment.h \
    #../VisionOld/GoalDetection.h \
    LayerSelectionWidget.h \
    locWmGlDisplay.h \
    #../VisionOld/LineDetection.h \
    ../Tools/Math/LSFittedLine.h \
    ../Tools/Math/Vector3.h \
    ../Infrastructure/FieldObjects/StationaryObject.h \
    ../Infrastructure/FieldObjects/Self.h \
    ../Infrastructure/FieldObjects/Object.h \
    ../Infrastructure/FieldObjects/MobileObject.h \
    ../Infrastructure/FieldObjects/AmbiguousObject.h \
    ../Infrastructure/FieldObjects/FieldObjects.h \
    ../Vision/Threads/SaveImagesThread.h \
    #../VisionOld/ObjectCandidate.h \
    ../Localisation/WMPoint.h \
    ../Localisation/WMLine.h \
    ../Localisation/sphere.h \
    ../Localisation/cylinder.h \
    ../Localisation/cameramatrix.h \
    ../Tools/Math/matrix.h \
    localisationwidget.h \
    #../VisionOld/Ball.h \
    #../VisionOld/CircleFitting.h \
    FileAccess/LogFileFormatReader.h \
    FileAccess/nifVersion1FormatReader.h \
    FileAccess/LogFileReader.h \
    FileAccess/nulVersion1FormatReader.h \
    visionstreamwidget.h \
    camerasettingswidget.h \
    ../NUPlatform/NUCamera/CameraSettings.h \
    ../Tools/FileFormats/Parse.h \
    ../Infrastructure/FieldObjects/WorldModelShareObject.h \
    ../Infrastructure/GameInformation/GameInformation.h \
    ../Tools/Threading/Thread.h \
    ../Tools/Threading/ConditionalThread.h \
    ../Tools/Threading/PeriodicThread.h \
    NUViewIO/NUViewIO.h \
    ../Kinematics/Kinematics.h \
    ../Tools/Math/TransformMatrices.h \
    frameInformationWidget.h \
    ../Tools/Math/SRUKF.h \
    ../Kinematics/Link.h \
    ../Kinematics/EndEffector.h \
    ../NUPlatform/NUSensors.h \
    ../Infrastructure/NUSensorsData/NUSensorsData.h \
    ../Infrastructure/NUData.h \
    ../Infrastructure/NUBlackboard.h \
    ../NUPlatform/NUPlatform.h \
    ../Tools/Math/General.h \
    ../VisionOld/CornerPoint.h \
    ../Kinematics/OrientationUKF.h \
    FileAccess/StreamFileReader.h \
    ../Tools/FileFormats/TimestampedData.h \
    FileAccess/ImageStreamFileReader.h \
    ../Motion/Tools/MotionScript.h \
    ../Motion/Tools/MotionCurves.h \
    ../Motion/Kicks/MotionScript2013.h \
#    ../VisionOld/EllipseFit.h \
#    ../VisionOld/EllipseFitting/tnt_version.h \
#    ../VisionOld/EllipseFitting/tnt_vec.h \
#    ../VisionOld/EllipseFitting/tnt_subscript.h \
#    ../VisionOld/EllipseFitting/tnt_stopwatch.h \
#    ../VisionOld/EllipseFitting/tnt_sparse_matrix_csr.h \
#    ../VisionOld/EllipseFitting/tnt_math_utils.h \
#    ../VisionOld/EllipseFitting/tnt_i_refvec.h \
#    ../VisionOld/EllipseFitting/tnt_fortran_array3d_utils.h \
#    ../VisionOld/EllipseFitting/tnt_fortran_array3d.h \
#    ../VisionOld/EllipseFitting/tnt_fortran_array2d_utils.h \
#    ../VisionOld/EllipseFitting/tnt_fortran_array2d.h \
#    ../VisionOld/EllipseFitting/tnt_fortran_array1d_utils.h \
#    ../VisionOld/EllipseFitting/tnt_fortran_array1d.h \
#    ../VisionOld/EllipseFitting/tnt_cmat.h \
#    ../VisionOld/EllipseFitting/tnt_array3d_utils.h \
#    ../VisionOld/EllipseFitting/tnt_array3d.h \
#    ../VisionOld/EllipseFitting/tnt_array2d_utils.h \
#    ../VisionOld/EllipseFitting/tnt_array2d.h \
#    ../VisionOld/EllipseFitting/tnt_array1d_utils.h \
#    ../VisionOld/EllipseFitting/tnt_array1d.h \
#    ../VisionOld/EllipseFitting/tnt.h \
#    ../VisionOld/EllipseFitting/jama_svd.h \
#    ../VisionOld/EllipseFitting/jama_qr.h \
#    ../VisionOld/EllipseFitting/jama_lu.h \
#    ../VisionOld/EllipseFitting/jama_eig.h \
#    ../VisionOld/EllipseFitting/jama_cholesky.h \
    FileAccess/SplitStreamFileFormatReader.h \
    SensorDisplayWidget.h \
    locwmstreamwidget.h \
    #../VisionOld/EllipseFitting/FittingCalculations.h \
    ../Tools/Math/Rectangle.h \
    ../NUPlatform/NUCamera.h \
    #../VisionOld/fitellipsethroughcircle.h \
    ../Localisation/LocWmFrame.h \
    FileAccess/IndexedFileReader.h \
    LUTGlDisplay.h \
    ../NUPlatform/NUSensors/EndEffectorTouch.h \
    ../NUPlatform/NUSensors/OdometryEstimator.h \
    ../Tools/Math/StlVector.h \
    ../Tools/Profiling/Profiler.h \
    MotionWidgets/WalkParameterWidget.h \
    MotionWidgets/KickWidget.h \
    MotionWidgets/MotionFileEditor.h \
    MotionWidgets/MotionFileSyntaxHighlighter.h \
    ../Motion/Walks/WalkParameters.h \
    ../Tools/Optimisation/Parameter.h \
    ../Tools/Math/FieldCalculations.h \
    ../NUPlatform/NUSensors/OdometryEstimator.h \
    OfflineLocalisation.h \
    ObjectDisplayWidget.h \
    TeamInformationDisplayWidget.h \
    GameInformationDisplayWidget.h \
    ../Infrastructure/TeamInformation/TeamInformation.h \
    ../Tools/FileFormats/LogRecorder.h \
    ../Tools/FileFormats/FileFormatException.h \
    offlinelocalisationdialog.h \
    ../Tools/Math/MultivariateGaussian.h \
    ../Localisation/SelfLocalisation.h \
    ../Localisation/MeasurementError.h \
    ../Localisation/SelfLocalisationTests.h \
    OfflineLocalisationSettingsDialog.h \
    LocalisationPerformanceMeasure.h \
    ../Localisation/LocalisationSettings.h \
    ../Tools/KFTools.h \
    ../NUPlatform/NUCamera/NUCameraData.h \
    ../Tools/Math/statistics.h \
    OfflineLocBatch.h \
    ../Localisation/Filters/UnscentedTransform.h \
    ../Localisation/Filters/UKF.h \
    ../Localisation/Filters/MobileObjectUKF.h \
    ../Tools/Math/depUKF.h \
    ../Localisation/iotests.h \
    NUViewConfig/*.h \
    plotdisplay.h \
    plotselectionwidget.h \
    createqwtsymboldialog.h \
    ../ConfigSystem/ConfigManager.h \
    ../ConfigSystem/ConfigParameter.h \
    ../ConfigSystem/ConfigRange.h \
    ../ConfigSystem/ConfigStorageManager.h \
    ../ConfigSystem/ConfigTree.h \
    ../ConfigSystem/Configurable.h \
    ../Localisation/Filters/IKalmanFilter.h \
    ../Localisation/Filters/IKFModel.h \
    ../Localisation/Filters/MobileObjectModel.h \
    ../Localisation/Filters/RobotModel.h \
    ../Localisation/Filters/KFBuilder.h \
    BatchSelectDialog.h \
    SensorCalibrationWidget.h \
    ../Localisation/Filters/WBasicUKF.h \
    ../Localisation/Filters/WSeqUKF.h \
    ../Localisation/Filters/SeqUKF.h \
    ../Localisation/Filters/WSrSeqUKF.h \
    ../Localisation/Filters/WSrBasicUKF.h \
    ../Localisation/Filters/IMUModel.h \
    ../Infrastructure/SensorCalibration.h

!win32 {
    HEADERS +=     ConnectionManager/ConnectionManager.h \
    ConnectionManager/BonjourProvider.h \
    ConnectionManager/BonjourServiceBrowser.h \
    ConnectionManager/BonjourServiceResolver.h \
    ConnectionManager/NUHostInfo.h \
    ConnectionManager/RobotSelectDialog.h
}

SOURCES += mainwindow.cpp \
    main.cpp \
    connectionwidget.cpp \
    classificationwidget.cpp \
    ../Tools/FileFormats/NUbotImage.cpp \
    #../VisionOld/Vision.cpp \
    ../Tools/FileFormats/LUTTools.cpp \
    virtualnubot.cpp \
    ../Infrastructure/NUImage/BresenhamLine.cpp \
    ../Tools/Math/Line.cpp \
    ../Tools/Math/Circle.cpp \
    ../Kinematics/Horizon.cpp \
    openglmanager.cpp \
    GLDisplay.cpp \
    ../Infrastructure/NUImage/NUImage.cpp \
    ../Infrastructure/NUImage/ClassifiedImage.cpp \
    #../VisionOld/ClassifiedSection.cpp \
    #../VisionOld/ScanLine.cpp \
    #../VisionOld/TransitionSegment.cpp \
    #../VisionOld/GoalDetection.cpp \
    LayerSelectionWidget.cpp \
    ../Motion/Tools/MotionFileTools.cpp \
    ../NUPlatform/NUIO.cpp \
    $$files(../NUPlatform/NUIO/*.cpp) \
    NUViewIO/NUViewIO.cpp \
    ../Infrastructure/NUData.cpp \
    ../Infrastructure/NUBlackboard.cpp \
    ../NUPlatform/NUPlatform.cpp \
    ../NUPlatform/NUSensors.cpp \
    $$files(../Infrastructure/NUSensorsData/*.cpp) \
    ../NUPlatform/NUActionators.cpp \
    $$files(../NUPlatform/NUActionators/*.cpp) \
    $$files(../Infrastructure/NUActionatorsData/*.cpp) \
    ../Infrastructure/TeamInformation/TeamInformation.cpp \
    $$files(../Infrastructure/Jobs/*.cpp) \
    $$files(../Infrastructure/Jobs/CameraJobs/*.cpp) \
    $$files(../Infrastructure/Jobs/VisionJobs/*.cpp) \
    $$files(../Infrastructure/Jobs/MotionJobs/*.cpp) \
    locWmGlDisplay.cpp \
    #../VisionOld/ObjectCandidate.cpp \
    #../VisionOld/LineDetection.cpp \
    ../Tools/Math/LSFittedLine.cpp \
    ../Infrastructure/FieldObjects/StationaryObject.cpp \
    ../Infrastructure/FieldObjects/Self.cpp \
    ../Infrastructure/FieldObjects/Object.cpp \
    ../Infrastructure/FieldObjects/MobileObject.cpp \
    ../Infrastructure/FieldObjects/AmbiguousObject.cpp \
    ../Infrastructure/FieldObjects/FieldObjects.cpp \
    ../Vision/Threads/*.cpp \
    ../Localisation/WMPoint.cpp \
    ../Localisation/WMLine.cpp \
    ../Localisation/sphere.cpp \
    ../Localisation/cylinder.cpp \
    ../Localisation/cameramatrix.cpp \
    ../Tools/Math/depUKF.cpp \
    ../Tools/Math/matrix.cpp \
    localisationwidget.cpp \
    #../VisionOld/Ball.cpp \
    #../VisionOld/CircleFitting.cpp \
    FileAccess/LogFileFormatReader.cpp \
    FileAccess/nifVersion1FormatReader.cpp \
    FileAccess/LogFileReader.cpp \
    FileAccess/nulVersion1FormatReader.cpp \
    visionstreamwidget.cpp \
    camerasettingswidget.cpp \
    ../NUPlatform/NUCamera/CameraSettings.cpp \
    ../Tools/FileFormats/Parse.cpp \
    ../Infrastructure/FieldObjects/WorldModelShareObject.cpp \
    ../Infrastructure/GameInformation/GameInformation.cpp \
    ../Tools/Threading/Thread.cpp \
    ../Tools/Threading/ConditionalThread.cpp \
    ../Tools/Threading/PeriodicThread.cpp \
    ../Kinematics/Kinematics.cpp \
    ../Tools/Math/TransformMatrices.cpp \
    frameInformationWidget.cpp \
    ../Tools/Math/SRUKF.cpp \
    ../Kinematics/Link.cpp \
    ../Kinematics/EndEffector.cpp \
    ../Kinematics/OrientationUKF.cpp \
    ../Motion/Tools/MotionScript.cpp \
    ../Motion/Tools/MotionCurves.cpp \
    ../Motion/Kicks/MotionScript2013.cpp \
    #../VisionOld/EllipseFit.cpp \
    FileAccess/SplitStreamFileFormatReader.cpp \
    SensorDisplayWidget.cpp \
    locwmstreamwidget.cpp \
    #../VisionOld/EllipseFitting/FittingCalculations.cpp \
    ../Tools/Math/Rectangle.cpp \
    ../NUPlatform/NUCamera.cpp \
    #../VisionOld/fitellipsethroughcircle.cpp \
    FileAccess/IndexedFileReader.cpp \
    LUTGlDisplay.cpp \
    ../NUPlatform/NUSensors/EndEffectorTouch.cpp \
    ../Tools/Math/FieldCalculations.cpp \
    ../NUPlatform/NUSensors/OdometryEstimator.cpp \
    ../Tools/Profiling/Profiler.cpp \
    MotionWidgets/WalkParameterWidget.cpp \
    MotionWidgets/KickWidget.cpp \
    MotionWidgets/MotionFileEditor.cpp \
    MotionWidgets/MotionFileSyntaxHighlighter.cpp \
    ../Motion/Walks/WalkParameters.cpp \
    ../Tools/Optimisation/Parameter.cpp \
    OfflineLocalisation.cpp \
    ObjectDisplayWidget.cpp \
    TeamInformationDisplayWidget.cpp \
    GameInformationDisplayWidget.cpp \
    ../Tools/FileFormats/LogRecorder.cpp \
    offlinelocalisationdialog.cpp \
    ../Tools/Math/MultivariateGaussian.cpp \
    ../Localisation/SelfLocalisation.cpp \
    ../Localisation/MeasurementError.cpp \
    ../Localisation/SelfLocalisationtests.cpp \
    OfflineLocalisationSettingsDialog.cpp \
    LocalisationPerformanceMeasure.cpp \
    ../Localisation/LocalisationSettings.cpp \
    ../Tools/KFTools.cpp \
    ../NUPlatform/NUCamera/NUCameraData.cpp \
    ../Tools/Math/statistics.cpp \
    OfflineLocBatch.cpp \
    ../Localisation/Filters/UKF.cpp \
    ../Localisation/Filters/MobileObjectUKF.cpp \
    ../Localisation/iotests.cpp \
    plotdisplay.cpp \
    plotselectionwidget.cpp \
    createqwtsymboldialog.cpp \
    ../ConfigSystem/ConfigManager.cpp \
    ../ConfigSystem/ConfigParameter.cpp \
    ../ConfigSystem/ConfigStorageManager.cpp \
    ../ConfigSystem/ConfigTree.cpp \
    ../ConfigSystem/Configurable.cpp \
    ../Localisation/Filters/MobileObjectModel.cpp \
    ../Localisation/Filters/RobotModel.cpp \
    ../Localisation/Filters/KFBuilder.cpp \
    BatchSelectDialog.cpp \
    SensorCalibrationWidget.cpp \
    ../Localisation/Filters/WBasicUKF.cpp \
    ../Localisation/Filters/WSeqUKF.cpp \
    ../Localisation/Filters/SeqUKF.cpp \
    ../Localisation/Filters/WSrSeqUKF.cpp \
    ../Localisation/Filters/WSrBasicUKF.cpp \
    ../Localisation/Filters/IMUModel.cpp

!win32{
    SOURCES+= ConnectionManager/ConnectionManager.cpp \
    ConnectionManager/BonjourProvider.cpp \
    ConnectionManager/BonjourServiceBrowser.cpp \
    ConnectionManager/BonjourServiceResolver.cpp \
    ConnectionManager/RobotSelectDialog.cpp
}
    
HEADERS += \
    ../Vision/VisionTypes/*.h \
    ../Vision/VisionTypes/RANSACTypes/*.h \
    ../Vision/VisionTypes/VisionFieldObjects/*.h \
    ../Vision/VisionWrapper/datawrappercurrent.h \
    ../Vision/VisionWrapper/visioncontrolwrappernuview.h \
    ../Vision/VisionWrapper/datawrappernuview.h \
    ../Vision/VisionTools/lookuptable.h \
    ../Vision/VisionTools/classificationcolours.h \
    ../Vision/VisionTools/transformer.h \
    ../Vision/Modules/*.h \
    ../Vision/Modules/LineDetectionAlgorithms/*.h \
    ../Vision/Modules/GoalDetectionAlgorithms/*.h \
    ../Vision/Modules/BallDetectionAlgorithms/*.h \
    ../Vision/*.h \

SOURCES += \
    ../Vision/VisionTypes/*.cpp \
    ../Vision/VisionTypes/RANSACTypes/*.cpp \
    ../Vision/VisionTypes/VisionFieldObjects/*.cpp \
    ../Vision/VisionTools/lookuptable.cpp \
    ../Vision/VisionTools/classificationcolours.cpp \
    ../Vision/VisionTools/transformer.cpp \
    ../Vision/Modules/*.cpp \
    ../Vision/Modules/LineDetectionAlgorithms/*.cpp \
    ../Vision/Modules/GoalDetectionAlgorithms/*.cpp \
    ../Vision/Modules/BallDetectionAlgorithms/*.cpp \
    ../Vision/visionblackboard.cpp \
    ../Vision/visioncontroller.cpp \
    ../Vision/visionconstants.cpp \
    ../Vision/basicvisiontypes.cpp \
    ../Vision/VisionWrapper/visioncontrolwrappernuview.cpp \
    ../Vision/VisionWrapper/datawrappernuview.cpp \

# pccamera uses Video4Linux, so only works on linux systems.
!macx{
    !win32{
        HEADERS += ../Vision/VisionTools/pccamera.h \
        SOURCES += ../Vision/VisionTools/pccamera.cpp \
    }
}

RESOURCES = Resources/textures.qrc Resources/icons.qrc Resources/styles.qrc
FORMS += \
    OfflineLocalisationSettingsDialog.ui \
    createqwtsymboldialog.ui \
    SensorCalibrationWidget.ui

