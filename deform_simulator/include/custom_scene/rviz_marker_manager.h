#pragma once

#include <mutex>
#include <unordered_map>

#include <QMainWindow>
#include <QTreeView>
#include <QStandardItemModel>
#include <QCheckBox>
#include <QPushButton>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include "custom_scene.h"

class RVizMarkerManager : public QMainWindow
{
    Q_OBJECT

private:

    struct IdRow
    {
    public:
        QStandardItem* id_item_;
        QStandardItem* enable_placeholder_;
        QStandardItem* delete_placeholder_;
        QCheckBox* enable_checkbox_;
        QPushButton* delete_button_;
    };

    struct NamespaceRow
    {
    public:
        QStandardItem* ns_item_;
        QCheckBox* enable_checkbox_;
        QPushButton* delete_button_;
        std::unordered_map<int32_t, IdRow> ids_;
    };


public:
    typedef std::shared_ptr<RVizMarkerManager> Ptr;

    explicit RVizMarkerManager(
            ros::NodeHandle& nh,
            CustomScene &scene,
            QWidget* parent = nullptr);
    ~RVizMarkerManager();

    // Threadsafe - locks and then emits a signal for the QT slot
    void visualizationMarkerCallback(const visualization_msgs::Marker& marker);
    void visualizationMarkerArrayCallback(const visualization_msgs::MarkerArray& marker_array);

    // Threadsafe - locks and then calls the internal handler
    bool clearVisualizationsCallback(
            std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &res);

protected:
    virtual void resizeEvent(QResizeEvent* event);

private slots:
    // Signaled only by qtMarkerSignal below
    void qtMarkerSlot();

private:
    void enableClicked(const std::string& ns);
    void enableClicked(const std::string& ns, const int32_t id);
    void deleteClicked(const std::string& ns);
    void deleteClicked(const std::string& ns, const int32_t id);

signals:
    // Used by ROS callbacks to signal the qtMarkerSlot above
    void qtMarkerSignal();

private:
    // Not threadsafe
    bool addOsgMarker(visualization_msgs::Marker marker);
    void deleteOsgMarker(const visualization_msgs::Marker& marker);
    void muteOsgMarker(const std::string ns, const int32_t id);
    void unmuteOsgMarker(const std::string ns, const int32_t id);
    void clearVisualizationsOsgHandler();

    // Not threadsafe
    void addQtMarker(const visualization_msgs::Marker& marker);
    void deleteQtMarker(const visualization_msgs::Marker& marker);
    void clearVisualizationsQtHandler();

    void addQtNamespace(const std::string& ns);
    void addQtIdInNamespace(const std::string& ns, const int32_t id);


private:
    ros::NodeHandle nh_;
    CustomScene& scene_;

    // ROS communictions handlers
    ros::Subscriber visualization_marker_sub_;
    ros::Subscriber visualization_marker_array_sub_;
    ros::ServiceServer clear_visualizations_srv_;

    std::mutex internal_data_mtx_;
    std::queue<visualization_msgs::Marker> marker_queue_;

    // QT model/view objects
    QTreeView view_;
    QStandardItemModel model_;
    std::unordered_map<std::string, NamespaceRow> namespace_to_model_items_;

    static constexpr auto TITLE = "Bullet Marker Visualization Management";
    static constexpr int STARTING_XPOS = 900;
    static constexpr int STARTING_YPOS = 10;
    static constexpr int STARTING_WIDTH = 450;
    static constexpr int STARTING_HEIGHT = 900;

    static constexpr int NAMESPACE_IDX = 0;
    static constexpr int ID_IDX = 0;
    static constexpr int ENABLE_IDX = 1;
    static constexpr int DELETE_IDX = 2;
    static constexpr int COLS = 3;

    static constexpr int NAMESPACE_WIDTH = 348;
    static constexpr int ENABLE_WIDTH = 25;
    static constexpr int DELETE_WIDTH = 75;

    // OSG pointers
    std::unordered_map<std::string, PlotLines::Ptr> visualization_line_markers_;
    std::unordered_map<std::string, PlotPoints::Ptr> visualization_point_markers_;
    std::unordered_map<std::string, PlotSpheres::Ptr> visualization_sphere_markers_;
    std::unordered_map<std::string, PlotBoxes::Ptr> visualization_box_markers_;
};
