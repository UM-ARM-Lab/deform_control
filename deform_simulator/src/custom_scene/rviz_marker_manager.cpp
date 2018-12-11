#include "custom_scene/rviz_marker_manager.h"

#include <deformable_manipulation_experiment_params/ros_params.hpp>
#include <bullet_helpers/bullet_ros_conversions.hpp>

using namespace smmap;
using namespace BulletHelpers;
namespace vm = visualization_msgs;

RVizMarkerManager::RVizMarkerManager(
        ros::NodeHandle& nh,
        CustomScene &scene,
        QWidget* parent)
    : QMainWindow(parent)
    , nh_(nh)
    , scene_(scene)
    , view_(this)
    , model_(0, COLS, this)
{
    assert(scene_.initialized());

    // Create the QT objects to handle marker management
    setWindowTitle(TITLE);
    setGeometry(STARTING_XPOS, STARTING_YPOS, STARTING_WIDTH, STARTING_HEIGHT);
    model_.setHorizontalHeaderLabels({"Namesapce/ID", "Enabled", "Delete"});
    view_.setModel(&model_);
    view_.setAlternatingRowColors(true);
    view_.setSortingEnabled(true);
    view_.setColumnWidth(NAMESPACE_IDX, NAMESPACE_WIDTH);
    view_.setColumnWidth(ENABLE_IDX, ENABLE_WIDTH);
    view_.setColumnWidth(DELETE_IDX, DELETE_WIDTH);
    view_.resize(this->size());

    // Whenver the signal "qtMarkerSignal" is emited, call the "qtMarkerSlot" function
    QObject::connect(this, &RVizMarkerManager::qtMarkerSignal, this, &RVizMarkerManager::qtMarkerSlot);

    // Create a subscriber to take visualization instructions
    visualization_marker_sub_ = nh_.subscribe(
            GetVisualizationMarkerTopic(nh_), 20, &RVizMarkerManager::visualizationMarkerCallback, this);

    // Create a subscriber to take visualization instructions
    visualization_marker_array_sub_ = nh_.subscribe(
            GetVisualizationMarkerArrayTopic(nh_), 20, &RVizMarkerManager::visualizationMarkerArrayCallback, this);

    // Create a service to clear all visualizations that have been sent to us via ROS
    clear_visualizations_srv_ = nh_.advertiseService(
            GetClearVisualizationsTopic(nh_), &RVizMarkerManager::clearVisualizationsCallback, this);
}

RVizMarkerManager::~RVizMarkerManager()
{}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks - These just invoke the internal handlers after locking
////////////////////////////////////////////////////////////////////////////////

void RVizMarkerManager::visualizationMarkerCallback(
        const vm::Marker& marker)
{
    if (!scene_.drawingOn)
    {
        return;
    }

    std::lock_guard<std::mutex> marker_queue_lock(internal_data_mtx_);
    marker_queue_.push(marker);
    emit qtMarkerSignal();
}

void RVizMarkerManager::visualizationMarkerArrayCallback(
        const vm::MarkerArray& marker_array)
{
    if (!scene_.drawingOn)
    {
        return;
    }

    std::lock_guard<std::mutex> marker_queue_lock(internal_data_mtx_);
    for (vm::Marker marker: marker_array.markers)
    {
        marker_queue_.push(marker);
    }
    emit qtMarkerSignal();
}

bool RVizMarkerManager::clearVisualizationsCallback(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res)
{
    if (!scene_.drawingOn)
    {
        return true;
    }

    (void)req;
    (void)res;

    // Clear by adding a deleteAll marker to the queue rather than handling the QT directly
    vm::Marker marker;
    marker.action = vm::Marker::DELETEALL;
    visualizationMarkerCallback(marker);
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// QT Callbacks
////////////////////////////////////////////////////////////////////////////////

void RVizMarkerManager::qtMarkerSlot()
{
    std::lock_guard<std::mutex> marker_queue_lock(internal_data_mtx_);
    if (!marker_queue_.empty())
    {
        std::lock_guard<std::mutex> lock(scene_.sim_mutex_);
        while(!marker_queue_.empty())
        {
            const auto& marker = marker_queue_.front();
            switch (marker.action)
            {
                // ADD and MODIFY now share the same enum, they did not used to do so
                case vm::Marker::ADD:
        //        case vm::Marker::MODIFY:
                    addQtMarker(marker);
                    addOsgMarker(marker);
                    break;

                case vm::Marker::DELETE:
//                    deleteQtMarker(marker);
                    deleteOsgMarker(marker);
                    break;

                case vm::Marker::DELETEALL:
//                    clearVisualizationsQtHandler();
                    clearVisualizationsOsgHandler();
                    break;

                default:
                    ROS_ERROR_STREAM_NAMED("visualization", "Unknown marker action " << marker.action);
                    break;
            }
            marker_queue_.pop();
        }
    }
}

void RVizMarkerManager::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    view_.resize(this->size());
}

////////////////////////////////////////////////////////////////////////////////
// Internal OSG handlers used by qtMarkerSlot() - not threadsafe
////////////////////////////////////////////////////////////////////////////////

// Returns true if a marker was added, false otherwise
bool RVizMarkerManager::addOsgMarker(
        vm::Marker marker)
{
    const std::string unique_id = marker.ns + std::to_string(marker.id);
    const bool is_active = namespace_to_model_items_[marker.ns].ids_[marker.id].enable_checkbox_->isChecked();

    // Transform the data into bullet's native frame (still world units)
    marker.pose = scene_.transformPoseToBulletFrame(marker.header, marker.pose).pose;
    marker.header.frame_id = scene_.bullet_frame_name_;

    switch (marker.type)
    {
        case vm::Marker::POINTS:
        {
            const auto marker_itr = visualization_point_markers_.find(unique_id);
            if (marker_itr == visualization_point_markers_.end())
            {
                PlotPoints::Ptr points = boost::make_shared<PlotPoints>();
                points->setPoints(toOsgRefVec3Array(scene_.world_to_bullet_tf_, marker.pose, marker.points, METERS),
                                  toOsgRefVec4Array(marker.colors));
                visualization_point_markers_[unique_id] = points;
                if (is_active)
                {
                    scene_.env->add(points);
                }
            }
            else
            {
                PlotPoints::Ptr points = marker_itr->second;
                points->setPoints(toOsgRefVec3Array(scene_.world_to_bullet_tf_, marker.pose, marker.points, METERS),
                                  toOsgRefVec4Array(marker.colors));
            }
            break;
        }
        case vm::Marker::CUBE:
        {
            if (marker.points.size() != 0)
            {
                marker.points.clear();
                ROS_WARN_STREAM_ONCE_NAMED("visualization", "Plotting a CUBE type that has the points field populated, the points field is only used for SPHERE/CUBE/LINE_LIST types. Namespace and id: " << unique_id);
            }
            marker.points.push_back(geometry_msgs::Point());
            marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), marker.color);
        }
        case vm::Marker::CUBE_LIST:
        {
            if (marker.colors.size() == 0)
            {
                marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), marker.color);
            }
            else if (marker.colors.size() != marker.points.size())
            {
                ROS_ERROR_STREAM_ONCE_NAMED("visualizer", "Marker colors field and points field have mismatching data sizes. Replacing contents of colors with the first element. Namespace and id: " << unique_id);
                marker.color = marker.colors[0];
                marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), marker.color);
            }

            // We don't have an "update" function for boxes, so delete any existing markers with the same unique_id
            if (is_active)
            {
                const auto marker_itr = visualization_box_markers_.find(unique_id);
                if (marker_itr != visualization_box_markers_.end())
                {
                    auto old_boxes = marker_itr->second;
                    scene_.env->remove(old_boxes);
                }
            }

            auto boxes = boost::make_shared<PlotBoxes>();
            boxes->plot(toOsgRefVec3Array(scene_.world_to_bullet_tf_, marker.pose, marker.points, METERS),
                        toOsgQuat(marker.pose.orientation),
                        toOsgRefVec4Array(marker.colors),
                        toOsgVec3(marker.scale, METERS));
            visualization_box_markers_[unique_id] = boxes;
            if (is_active)
            {
                scene_.env->add(boxes);
            }
            break;
        }
        case vm::Marker::SPHERE:
        {
            if (marker.points.size() != 0)
            {
                marker.points.clear();
                ROS_WARN_STREAM_ONCE_NAMED("visualization", "Plotting a SPHERE type that has the points field populated, the points field is only used for SPHERE/CUBE/LINE_LIST types. Namespace and id: " << unique_id);
            }
            if (marker.colors.size() != marker.points.size())
            {
                marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), marker.color);
            }
        }
        case vm::Marker::SPHERE_LIST:
        {
            if (marker.colors.size() == 0)
            {
                marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), marker.color);
            }
            else if (marker.colors.size() != marker.points.size())
            {
                ROS_ERROR_STREAM_ONCE_NAMED("visualizer", "Marker colors field and points field have mismatching data sizes. Replacing contents of colors with the first element. Namespace and id: " << unique_id);
                marker.color = marker.colors[0];
                marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), marker.color);
            }

            if ((marker.scale.y != marker.scale.x && marker.scale.y != 0.0f) ||
                (marker.scale.z != marker.scale.x && marker.scale.z != 0.0f))
            {
                ROS_WARN_STREAM_ONCE_NAMED("visualization", "Plotting a SPHERE_LIST type that meaningful data in the y or z fields. This data is ignored. Namespace and id: " << unique_id);
            }

            const auto marker_itr = visualization_sphere_markers_.find(unique_id);
            if (marker_itr == visualization_sphere_markers_.end())
            {
                auto spheres = boost::make_shared<PlotSpheres>();
                spheres->plot(toOsgRefVec3Array(scene_.world_to_bullet_tf_, marker.pose, marker.points, METERS),
                              toOsgRefVec4Array(marker.colors),
                              // Note that we are converting from a diameter to a radius here
                              std::vector<float>(marker.points.size(), (float)marker.scale.x * 0.5f * METERS));
                visualization_sphere_markers_[unique_id] = spheres;
                if (is_active)
                {
                    scene_.env->add(spheres);
                }
            }
            else
            {
                auto spheres = marker_itr->second;
                spheres->plot(toOsgRefVec3Array(scene_.world_to_bullet_tf_, marker.pose, marker.points, METERS),
                              toOsgRefVec4Array(marker.colors),
                              // Note that we are converting from a diameter to a radius here
                              std::vector<float>(marker.points.size(), (float)marker.scale.x * 0.5f * METERS));
            }
            break;
        }
        case vm::Marker::LINE_STRIP:
        {
            if (marker.points.size() == 1)
            {
                ROS_ERROR_STREAM_NAMED(
                            "visualization",
                            "Only 1 point for line strip:\n"
                            << "  NS: " << marker.ns << std::endl
                            << "  ID: " << marker.id << std::endl
                            << "  Point: " << marker.points[0].x << " " << marker.points[0].y << " " << marker.points[0].z);
            }
            if (marker.points.size() > 1)
            {
                convertLineStripToLineList(marker);
            }
        }
        case vm::Marker::LINE_LIST:
        {
            const auto marker_itr = visualization_line_markers_.find(unique_id);
            if (marker_itr == visualization_line_markers_.end())
            {
                // Scale factor of 100 is arbitrary; this is used because "linewidth"
                // in the construtor is actually "glLineWidth" which measures things in pixels
                auto line_strip = boost::make_shared<PlotLines>((float)marker.scale.x * METERS * 100);
                // marker.pose has already be transformed to bullet's native frame
                line_strip->setPoints(toBulletPointVector(marker.pose, marker.points, METERS),
                                      toBulletColorArray(marker.colors));
                visualization_line_markers_[unique_id] = line_strip;
                if (is_active)
                {
                    scene_.env->add(line_strip);
                }
            }
            else
            {
                auto line_strip = marker_itr->second;
                // marker.pose has already be transformed to bullet's native frame
                line_strip->setPoints(toBulletPointVector(marker.pose, marker.points, METERS),
                                      toBulletColorArray(marker.colors));
            }
            break;
        }
        default:
        {
            ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "visualization",
                    "Marker type " << marker.type << " not implemented " << unique_id);
            return false;
        }
    }
    return true;
}

void RVizMarkerManager::deleteOsgMarker(
        const vm::Marker& marker)
{
    const std::string unique_id = marker.ns + std::to_string(marker.id);

    // Delete any matching marker from the points list
    {
        const auto points_marker_itr = visualization_point_markers_.find(unique_id);
        if (points_marker_itr != visualization_point_markers_.end())
        {
            scene_.env->remove(points_marker_itr->second);
            visualization_point_markers_.erase(points_marker_itr);
        }
    }

    // Delete any matching marker from the spheres list
    {
        const auto sphere_marker_itr = visualization_sphere_markers_.find(unique_id);
        if (sphere_marker_itr != visualization_sphere_markers_.end())
        {
            scene_.env->remove(sphere_marker_itr->second);
            visualization_sphere_markers_.erase(sphere_marker_itr);
        }
    }

    // Delete any matching markers from the lines list
    {
        const auto line_marker_itr = visualization_line_markers_.find(unique_id);
        if (line_marker_itr != visualization_line_markers_.end())
        {
            scene_.env->remove(line_marker_itr->second);
            visualization_line_markers_.erase(line_marker_itr);
        }
    }

    // Delete any matching marker from the boxes list
    {
        const auto box_marker_itr = visualization_box_markers_.find(unique_id);
        if (box_marker_itr != visualization_box_markers_.end())
        {
            scene_.env->remove(box_marker_itr->second);
            visualization_box_markers_.erase(box_marker_itr);
        }
    }
}

void RVizMarkerManager::muteOsgMarker(
        const std::string ns,
        const int32_t id)
{
    const std::string unique_id = ns + std::to_string(id);

    // Find any matching points markers, and disable them in the visualizer
    {
        const auto points_marker_itr = visualization_point_markers_.find(unique_id);
        if (points_marker_itr != visualization_point_markers_.end())
        {
            scene_.env->remove(points_marker_itr->second);
        }
    }

    // Find any matching spheres markers, and disable them in the visualizer
    {
        const auto spheres_marker_itr = visualization_sphere_markers_.find(unique_id);
        if (spheres_marker_itr != visualization_sphere_markers_.end())
        {
            scene_.env->remove(spheres_marker_itr->second);
        }
    }

    // Find any matching lines markers, and disable them in the visualizer
    {
        const auto line_marker_itr = visualization_line_markers_.find(unique_id);
        if (line_marker_itr != visualization_line_markers_.end())
        {
            scene_.env->remove(line_marker_itr->second);
        }
    }

    // Find any matching boxes markers, and disable them in the visualizer
    {
        const auto box_marker_itr = visualization_box_markers_.find(unique_id);
        if (box_marker_itr != visualization_box_markers_.end())
        {
            scene_.env->remove(box_marker_itr->second);
        }
    }
}

void RVizMarkerManager::unmuteOsgMarker(
        const std::string ns,
        const int32_t id)
{
    const std::string unique_id = ns + std::to_string(id);

    // Find any matching points markers, and enable them in the visualizer
    {
        const auto points_marker_itr = visualization_point_markers_.find(unique_id);
        if (points_marker_itr != visualization_point_markers_.end())
        {
            scene_.env->add(points_marker_itr->second);
        }
    }

    // Find any matching spheres markers, and enable them in the visualizer
    {
        const auto spheres_marker_itr = visualization_sphere_markers_.find(unique_id);
        if (spheres_marker_itr != visualization_sphere_markers_.end())
        {
            scene_.env->add(spheres_marker_itr->second);
        }
    }

    // Find any matching lines markers, and enable them in the visualizer
    {
        const auto line_marker_itr = visualization_line_markers_.find(unique_id);
        if (line_marker_itr != visualization_line_markers_.end())
        {
            scene_.env->add(line_marker_itr->second);
        }
    }

    // Find any matching boxes markers, and enable them in the visualizer
    {
        const auto box_marker_itr = visualization_box_markers_.find(unique_id);
        if (box_marker_itr != visualization_box_markers_.end())
        {
            scene_.env->add(box_marker_itr->second);
        }
    }
}

void RVizMarkerManager::clearVisualizationsOsgHandler()
{
    // Delete the entire points list
    {
        for (auto& points_marker_pair : visualization_point_markers_)
        {
            scene_.env->remove(points_marker_pair.second);
        }
        visualization_point_markers_.clear();
    }

    // Delete the entire spheres list
    {
        for (auto& sphere_marker_pair : visualization_sphere_markers_)
        {
            scene_.env->remove(sphere_marker_pair.second);
        }
        visualization_sphere_markers_.clear();
    }

    // Delete the entire lines list
    {
        for (auto& line_marker_pair : visualization_line_markers_)
        {
            scene_.env->remove(line_marker_pair.second);
        }
        visualization_line_markers_.clear();
    }

    // Delete the entire boxes list
    {
        for (auto& box_marker_pair : visualization_box_markers_)
        {
            scene_.env->remove(box_marker_pair.second);
        }
        visualization_box_markers_.clear();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Internal QT handlers used by qtMarkerSlot() - not threadsafe
////////////////////////////////////////////////////////////////////////////////

void RVizMarkerManager::addQtMarker(const vm::Marker& marker)
{
    addQtNamespace(marker.ns);
    addQtIdInNamespace(marker.ns, marker.id);
}

void RVizMarkerManager::deleteQtMarker(const vm::Marker& marker)
{
//    NamespaceRow& ns_row = namespace_to_model_items_.at(marker.ns);
//    const IdRow& id_row = ns_row.ids_.at(marker.id);
//    ns_row.ns_item_->removeRow(id_row.id_item_->row());
//    ns_row.ids_.erase(ns_row.ids_.find(marker.id));
}

void RVizMarkerManager::clearVisualizationsQtHandler()
{
//    model_.removeRows(0, model_.rowCount());
//    namespace_to_model_items_.clear();
}

void RVizMarkerManager::addQtNamespace(const std::string& ns)
{
    // If the given namespace is not yet known, add a row to the model
    const auto itr = namespace_to_model_items_.find(ns);
    if (itr == namespace_to_model_items_.end())
    {
        // Create an item with a caption and add it to a new row in the model
        const auto ns_item = new QStandardItem(QString::fromStdString(ns));
        ns_item->setEditable(false);
        const auto row = model_.rowCount();
        model_.setItem(row, NAMESPACE_IDX, ns_item);

        // Create a checkbox and add it to new view at the new row
        const auto enable_box = new QCheckBox();
        enable_box->setChecked(true);
        enable_box->setTristate(true);
        auto enable_callback = [=] (bool checked)
        {
            (void)checked;
            enableClicked(ns);
        };
        QObject::connect(enable_box, &QCheckBox::clicked, enable_callback);
        const auto enable_idx = model_.index(row, ENABLE_IDX);
        view_.setIndexWidget(enable_idx, enable_box);

        // Create a button and add it to the new view at the new row
        const auto delete_button = new QPushButton("Delete");
        auto delete_callback = [=] (bool checked)
        {
            (void)checked;
            deleteClicked(ns);
        };
        QObject::connect(delete_button, &QPushButton::clicked, delete_callback);
        const auto delete_idx = model_.index(row, DELETE_IDX);
        view_.setIndexWidget(delete_idx, delete_button);

        // Record the new items into our stored data structures for easy lookup
        namespace_to_model_items_[ns] = {ns_item, enable_box, delete_button, {}};
    }
}

void RVizMarkerManager::addQtIdInNamespace(const std::string& ns, const int32_t id)
{
    // Add the namsspace if needed
    addQtNamespace(ns);
    NamespaceRow& ns_row = namespace_to_model_items_.at(ns);

    // If the given id is not yet known, add a row to the model
    const auto itr = ns_row.ids_.find(id);
    if (itr == ns_row.ids_.end())
    {
        // Create an item with a caption and add it to a new row in the model
        const auto id_item = new QStandardItem(QString::fromStdString(std::to_string(id)));
        id_item->setEditable(false);
        // Append 3 items to allow for indexing into the extra locations
        const auto enable_placeholder = new QStandardItem();
        const auto delete_placeholder = new QStandardItem();
        auto item_list = {id_item, enable_placeholder, delete_placeholder};
        ns_row.ns_item_->appendRow(item_list);

        // Create a checkbox and add it to new view at the new row
        const auto enable_box = new QCheckBox();
        const bool start_checked = (ns_row.enable_checkbox_->checkState() != Qt::CheckState::Unchecked);
        enable_box->setChecked(start_checked);
        auto enable_callback = [=] (bool checked)
        {
            (void)checked;
            enableClicked(ns, id);
        };
        QObject::connect(enable_box, &QCheckBox::clicked, enable_callback);
        const auto enable_idx = ns_row.ns_item_->child(id_item->row(), ENABLE_IDX)->index();
        view_.setIndexWidget(enable_idx, enable_box);

        // Create a button and add it to the new view at the new row
        const auto delete_button = new QPushButton("Delete");
        auto delete_callback = [=] (bool checked)
        {
            (void)checked;
            deleteClicked(ns, id);
        };
        QObject::connect(delete_button, &QPushButton::clicked, delete_callback);
        const auto delete_idx = ns_row.ns_item_->child(id_item->row(), DELETE_IDX)->index();
        view_.setIndexWidget(delete_idx, delete_button);

        // Record the new items into our stored data structures for easy lookup
        ns_row.ids_[id] = {id_item, enable_placeholder, delete_placeholder, enable_box, delete_button};
    }
}

////////////////////////////////////////////////////////////////////////////////
// Internal QT handlers used when Widgets are clicked
////////////////////////////////////////////////////////////////////////////////

// Applies to the whole namespace
void RVizMarkerManager::enableClicked(const std::string& ns)
{
    std::lock_guard<std::mutex> marker_queue_lock(internal_data_mtx_);
    std::lock_guard<std::mutex> lock(scene_.sim_mutex_);

    const NamespaceRow& ns_row = namespace_to_model_items_.at(ns);
    const auto checkbox = ns_row.enable_checkbox_;

    // Don't allow the checkbox to be clicked to the partial state,
    // instead bypass directly to fully checked
    if (checkbox->checkState() == Qt::CheckState::PartiallyChecked)
    {
        checkbox->setCheckState(Qt::CheckState::Checked);
    }

    if (checkbox->isChecked())
    {
        // Go through each item, check it, and add it to the visualization if needed
        for (const auto& data: ns_row.ids_)
        {
            const IdRow& id_row = data.second;
            id_row.enable_checkbox_->setChecked(true);
            unmuteOsgMarker(ns, id_row.id_item_->text().toInt());
        }
    }
    else
    {
        // Go through each item, uncheck it, and remove it from the visualization if needed
        for (const auto& data: ns_row.ids_)
        {
            const IdRow& id_row = data.second;
            id_row.enable_checkbox_->setChecked(false);
            muteOsgMarker(ns, id_row.id_item_->text().toInt());
        }
    }
}

// Applies only to a single id within the namespace
void RVizMarkerManager::enableClicked(const std::string& ns, const int32_t id)
{
    std::lock_guard<std::mutex> marker_queue_lock(internal_data_mtx_);
    std::lock_guard<std::mutex> lock(scene_.sim_mutex_);

    const NamespaceRow& ns_row = namespace_to_model_items_.at(ns);
    const IdRow& id_row = ns_row.ids_.at(id);
    const auto& checkbox = id_row.enable_checkbox_;

    if (checkbox->isChecked())
    {
        unmuteOsgMarker(ns, id_row.id_item_->text().toInt());
    }
    else
    {
        muteOsgMarker(ns, id_row.id_item_->text().toInt());
    }
}

// Applies to the whole namespace
void RVizMarkerManager::deleteClicked(const std::string& ns)
{
    std::lock_guard<std::mutex> marker_queue_lock(internal_data_mtx_);
    std::lock_guard<std::mutex> lock(scene_.sim_mutex_);

    const NamespaceRow& ns_row = namespace_to_model_items_.at(ns);

    // Remove it from OSG
    {
        vm::Marker marker;
        marker.ns = ns;
        for (const auto& id_row_pair : ns_row.ids_)
        {
            const int32_t id = id_row_pair.first;
            marker.id = id;
            deleteOsgMarker(marker);
        }
    }

    // Remove it from QT
    {
        model_.removeRow(ns_row.ns_item_->row());
        namespace_to_model_items_.erase(namespace_to_model_items_.find(ns));
    }
}

// Applies only to a single id within the namespace
void RVizMarkerManager::deleteClicked(const std::string& ns, const int32_t id)
{
    std::lock_guard<std::mutex> marker_queue_lock(internal_data_mtx_);
    std::lock_guard<std::mutex> lock(scene_.sim_mutex_);

    // Remove it from OSG
    {
        vm::Marker marker;
        marker.ns = ns;
        marker.id = id;
        deleteOsgMarker(marker);
    }

    // Remove it from QT
    {
        NamespaceRow& ns_row = namespace_to_model_items_.at(ns);
        const IdRow& id_row = ns_row.ids_.at(id);
        ns_row.ns_item_->removeRow(id_row.id_item_->row());
        ns_row.ids_.erase(ns_row.ids_.find(id));
    }
}
