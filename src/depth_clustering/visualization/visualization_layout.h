/*
 * visualization_layout.h
 *
 *  Created on: Oct 11, 2021
 *      Author: simonyu
 */

#ifndef DEPTH_CLUSTERING_VISUALIZATION_VISUALIZATION_LAYOUT_H_
#define DEPTH_CLUSTERING_VISUALIZATION_VISUALIZATION_LAYOUT_H_

#include <boost/property_tree/json_parser.hpp>
#include <iostream>

using boost::property_tree::json_parser::read_json;

namespace visualization
{
struct VisualizationLayout
{
	enum class PointCloudViewerLayer : int
	{
		None = 0,
		Ground_Removal,
		Second_Return,
		Intensity,
		Elongation,
		Point_Score,
		Cluster_Score,
		Frame_Score
	};

	enum class ImageViewerLayer : int
	{
		None = 0, Clustering, Segmentation, Range, Intensity, Elongation
	};

	enum class LidarReturn : int
	{
		First = 0, Second, Custom
	};

	enum class BoundingBoxType : int
	{
		None = 0, Cube, Polygon
	};

	enum class FieldOfView : int
	{
		Default = 0, Camera
	};

	PointCloudViewerLayer point_cloud_viewer_layer;
	ImageViewerLayer image_viewer_layer_left;
	ImageViewerLayer image_viewer_layer_middle;
	ImageViewerLayer image_viewer_layer_right;
	LidarReturn lidar_return;
	BoundingBoxType bounding_box_type;
	FieldOfView field_of_view;
	bool show_camera_frame;

	VisualizationLayout() :
			point_cloud_viewer_layer(PointCloudViewerLayer::None), image_viewer_layer_left(
					ImageViewerLayer::Clustering), image_viewer_layer_middle(
					ImageViewerLayer::Segmentation), image_viewer_layer_right(
					ImageViewerLayer::Range), lidar_return(LidarReturn::First), bounding_box_type(
					BoundingBoxType::Polygon), field_of_view(FieldOfView::Camera), show_camera_frame(
					true)
	{
	}

	inline void
	configure(const std::string& file_path_name_config_layout)
	{
		boost::property_tree::ptree tree;

		try
		{
			boost::property_tree::read_json(file_path_name_config_layout, tree);
		} catch (boost::exception const &e)
		{
			std::cout << "[ERROR]: Failed to load layout configuration file: \""
					<< file_path_name_config_layout << "\"." << std::endl;
			return;
		}

		auto point_cloud_viewer_layer_optional = tree.get_optional<std::string>(
				"point_cloud_viewer_layer");
		auto image_viewer_layer_left_optional = tree.get_optional<std::string>(
				"image_viewer_layer_left");
		auto image_viewer_layer_middle_optional = tree.get_optional<std::string>(
				"image_viewer_layer_middle");
		auto image_viewer_layer_right_optional = tree.get_optional<std::string>(
				"image_viewer_layer_right");
		auto lidar_return_optional = tree.get_optional<std::string>("lidar_return");
		auto bounding_box_type_optional = tree.get_optional<std::string>("bounding_box_type");
		auto field_of_view_optional = tree.get_optional<std::string>("field_of_view");
		auto show_camera_frame_optional = tree.get_optional<bool>("show_camera_frame");

		if (point_cloud_viewer_layer_optional)
		{
			const std::string point_cloud_viewer_layer_string = *point_cloud_viewer_layer_optional;

			if (point_cloud_viewer_layer_string == "none")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::None;
			}
			else if (point_cloud_viewer_layer_string == "ground_removal")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::Ground_Removal;
			}
			else if (point_cloud_viewer_layer_string == "second_return")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::Second_Return;
			}
			else if (point_cloud_viewer_layer_string == "intensity")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::Intensity;
			}
			else if (point_cloud_viewer_layer_string == "elongation")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::Elongation;
			}
			else if (point_cloud_viewer_layer_string == "point_score")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::Point_Score;
			}
			else if (point_cloud_viewer_layer_string == "cluster_score")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::Cluster_Score;
			}
			else if (point_cloud_viewer_layer_string == "frame_score")
			{
				point_cloud_viewer_layer = PointCloudViewerLayer::Frame_Score;
			}
			else
			{
				std::cout << "[WARN]: Unknown point cloud viewer layer: "
						<< point_cloud_viewer_layer_string << "." << std::endl;
			}
		}
		else
		{
			std::cout << "[WARN]: Point cloud viewer layer configuration missing." << std::endl;
		}

		if (image_viewer_layer_left_optional)
		{
			const std::string image_viewer_layer_left_string = *image_viewer_layer_left_optional;

			if (image_viewer_layer_left_string == "none")
			{
				image_viewer_layer_left = ImageViewerLayer::None;
			}
			else if (image_viewer_layer_left_string == "clustering")
			{
				image_viewer_layer_left = ImageViewerLayer::Clustering;
			}
			else if (image_viewer_layer_left_string == "segmentation")
			{
				image_viewer_layer_left = ImageViewerLayer::Segmentation;
			}
			else if (image_viewer_layer_left_string == "range")
			{
				image_viewer_layer_left = ImageViewerLayer::Range;
			}
			else if (image_viewer_layer_left_string == "intensity")
			{
				image_viewer_layer_left = ImageViewerLayer::Intensity;
			}
			else if (image_viewer_layer_left_string == "elongation")
			{
				image_viewer_layer_left = ImageViewerLayer::Elongation;
			}
			else
			{
				std::cout << "[WARN]: Unknown left image viewer layer: "
						<< image_viewer_layer_left_string << "." << std::endl;
			}
		}
		else
		{
			std::cout << "[WARN]: Left image viewer layer configuration missing." << std::endl;
		}

		if (image_viewer_layer_middle_optional)
		{
			const std::string image_viewer_layer_middle_string = *image_viewer_layer_middle_optional;

			if (image_viewer_layer_middle_string == "none")
			{
				image_viewer_layer_middle = ImageViewerLayer::None;
			}
			else if (image_viewer_layer_middle_string == "clustering")
			{
				image_viewer_layer_middle = ImageViewerLayer::Clustering;
			}
			else if (image_viewer_layer_middle_string == "segmentation")
			{
				image_viewer_layer_middle = ImageViewerLayer::Segmentation;
			}
			else if (image_viewer_layer_middle_string == "range")
			{
				image_viewer_layer_middle = ImageViewerLayer::Range;
			}
			else if (image_viewer_layer_middle_string == "intensity")
			{
				image_viewer_layer_middle = ImageViewerLayer::Intensity;
			}
			else if (image_viewer_layer_middle_string == "elongation")
			{
				image_viewer_layer_middle = ImageViewerLayer::Elongation;
			}
			else
			{
				std::cout << "[WARN]: Unknown middle image viewer layer: "
						<< image_viewer_layer_middle_string << "." << std::endl;
			}
		}
		else
		{
			std::cout << "[WARN]: Middle image viewer layer configuration missing." << std::endl;
		}

		if (image_viewer_layer_right_optional)
		{
			const std::string image_viewer_layer_right_string = *image_viewer_layer_right_optional;

			if (image_viewer_layer_right_string == "none")
			{
				image_viewer_layer_right = ImageViewerLayer::None;
			}
			else if (image_viewer_layer_right_string == "clustering")
			{
				image_viewer_layer_right = ImageViewerLayer::Clustering;
			}
			else if (image_viewer_layer_right_string == "segmentation")
			{
				image_viewer_layer_right = ImageViewerLayer::Segmentation;
			}
			else if (image_viewer_layer_right_string == "range")
			{
				image_viewer_layer_right = ImageViewerLayer::Range;
			}
			else if (image_viewer_layer_right_string == "intensity")
			{
				image_viewer_layer_right = ImageViewerLayer::Intensity;
			}
			else if (image_viewer_layer_right_string == "elongation")
			{
				image_viewer_layer_right = ImageViewerLayer::Elongation;
			}
			else
			{
				std::cout << "[WARN]: Unknown right image viewer layer: "
						<< image_viewer_layer_right_string << "." << std::endl;
			}
		}
		else
		{
			std::cout << "[WARN]: Right image viewer layer configuration missing." << std::endl;
		}

		if (lidar_return_optional)
		{
			const std::string lidar_return_string = *lidar_return_optional;

			if (lidar_return_string == "first")
			{
				lidar_return = LidarReturn::First;
			}
			else if (lidar_return_string == "second")
			{
				lidar_return = LidarReturn::Second;
			}
			else
			{
				std::cout << "[WARN]: Unknown lidar return: " << lidar_return_string << "."
						<< std::endl;
			}
		}
		else
		{
			std::cout << "[WARN]: Lidar return configuration missing." << std::endl;
		}

		if (bounding_box_type_optional)
		{
			const std::string bounding_box_type_string = *bounding_box_type_optional;

			if (bounding_box_type_string == "none")
			{
				bounding_box_type = BoundingBoxType::None;
			}
			else if (bounding_box_type_string == "cube")
			{
				bounding_box_type = BoundingBoxType::Cube;
			}
			else if (bounding_box_type_string == "polygon")
			{
				bounding_box_type = BoundingBoxType::Polygon;
			}
			else
			{
				std::cout << "[WARN]: Unknown bounding box type: " << bounding_box_type_string
						<< "." << std::endl;
			}
		}
		else
		{
			std::cout << "[WARN]: Bounding box type configuration missing." << std::endl;
		}

		if (field_of_view_optional)
		{
			const std::string field_of_view_string = *field_of_view_optional;

			if (field_of_view_string == "default")
			{
				field_of_view = FieldOfView::Default;
			}
			else if (field_of_view_string == "camera")
			{
				field_of_view = FieldOfView::Camera;
			}
			else
			{
				std::cout << "[WARN]: Unknown field of view: " << field_of_view_string << "."
						<< std::endl;
			}
		}
		else
		{
			std::cout << "[WARN]: Field of view configuration missing." << std::endl;
		}

		if (show_camera_frame_optional)
		{
			show_camera_frame = *show_camera_frame_optional;
		}
		else
		{
			std::cout << "[WARN]: Show camera frame configuration missing." << std::endl;
		}
	}
};
} // namespace visualization

#endif /* DEPTH_CLUSTERING_VISUALIZATION_VISUALIZATION_LAYOUT_H_ */
