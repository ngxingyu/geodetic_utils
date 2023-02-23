#include <geotf/geodetic_converter.h>
namespace geotf {

GeodeticConverter::GeodeticConverter(): Node("geodetic_converter") {
    if (GDAL_VERSION_MAJOR < 3) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Found GDAL Version < 3, performing axis swaps.");
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Found GDAL Version >=3, NOT performing axis swaps.");
    }
    const std::string &prefix = "geotf";
    Node::declare_parameter(prefix, rclcpp::PARAMETER_STRING);
    if (!this->has_parameter(prefix)) {
        RCLCPP_WARN(this->get_logger(), "[GeoTF] No GeodeticTF Transformations found.");
        return;
    }
    std::string yaml_raw_data = this->get_parameter(prefix).as_string();
    RCLCPP_INFO_ONCE(this->get_logger(), ("geotf:\n" + yaml_raw_data).c_str());

    try {
        YAML::Node geotf = YAML::Load(yaml_raw_data);
        geotf["Frames"];

        auto frame_definitions = geotf["Frames"].as<std::map<std::string, YAML::Node>>();
        for (auto it = frame_definitions.begin(); it != frame_definitions.end();
             ++it) {
            const std::string frame_name = it->first;
            auto xmlnode = it->second;

            std::string frame_type = xmlnode["Type"].as<std::string>();
            if (frame_type == "EPSGCode") {
                if (!xmlnode["Code"]) {
                    RCLCPP_WARN_STREAM(this->get_logger(), "[GeoTF] Ignoring frame "
                                                               << frame_type << ": EPSGCode needs Code setting.");
                    continue;
                }
                int code = xmlnode["Code"].as<int>();
                addFrameByEPSG(frame_name, code);

            } else if (frame_type == "GCSCode") {
                if (!xmlnode["Code"]) {
                    RCLCPP_WARN_STREAM(this->get_logger(), "[GeoTF] Ignoring frame "
                                                               << frame_type << ": GCSCode needs Code setting.");
                    continue;
                }
                std::string code = xmlnode["Code"].as<std::string>();
                addFrameByGCSCode(frame_name, code);

            } else if (frame_type == "UTM") {
                if (!xmlnode["Zone"] || !xmlnode["Hemisphere"]) {
                    RCLCPP_WARN_STREAM(this->get_logger(), "[GeoTF] Ignoring frame "
                                                               << frame_type
                                                               << ": UTM needs Zone and Hemisphere setting.");
                    continue;
                }
                int zone = xmlnode["Zone"].as<int>();
                bool hemisphere = xmlnode["Hemisphere"].as<std::string>() == "N";
                addFrameByUTM(frame_name, zone, hemisphere);

            } else if (frame_type == "ENUOrigin") {
                if (!xmlnode["LatOrigin"] || !xmlnode["LonOrigin"] ||
                    !xmlnode["AltOrigin"]) {
                    RCLCPP_WARN_STREAM(this->get_logger(),
                                       "[GeoTF] Ignoring frame "
                                           << frame_type
                                           << ": ENUOrigin needs LatOrigin, LonOrigin and AltOrigin setting.");
                    continue;
                }
                double latOrigin = xmlnode["LatOrigin"].as<double>();
                double lonOrigin = xmlnode["LonOrigin"].as<double>();
                double altOrigin = xmlnode["AltOrigin"].as<double>();

                addFrameByENUOrigin(frame_name, latOrigin, lonOrigin, altOrigin);
            }
        }

        // Get TF Mapping
        if (geotf["TF_Mapping"]) {
            auto tf_mapping = geotf["TF_Mapping"].as<std::map<std::string, std::string>>();
            std::string geo_tf = tf_mapping["GEO_TF"];
            std::string tf = tf_mapping["TF"];
            if (mappings_.count(geo_tf)) {
                tf_mapping_ = std::make_pair(geo_tf, tf);
            } else {
                RCLCPP_WARN_STREAM(this->get_logger(), "[GeoTF] Invalid Tf connection, frame "
                                                           << geo_tf << " not defined.");
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "[GeoTF] TF connection is " << geo_tf << " = " << tf);
        } else {
            RCLCPP_WARN_STREAM(this->get_logger(), "[GeoTF] No TF connection specified.");
        }

    } catch (const YAML::BadFile &e) {
        std::cerr << e.msg << std::endl;
        raise(SIGINT);
    } catch (const YAML::ParserException &e) {
        std::cerr << e.msg << std::endl;
        raise(SIGINT);
    }

    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

// Adds a coordinate frame by its EPSG identifier
// see https://spatialreference.org/ref/epsg/
// Example: CH1903+ has epsg id 2056
//          https://spatialreference.org/ref/epsg/2056/
bool GeodeticConverter::addFrameByEPSG(const std::string &name, const int &id) {
    if (mappings_.count(name)) {
        return false;
    }

    // Create Spatial Reference from EPSG ID
    auto spatial_ref = std::make_shared<OGRSpatialReference>();
    OGRErr err = spatial_ref->importFromEPSG(id);
    if (err != OGRERR_NONE) {
        std::cout << "ERROR" << err << std::endl;
        return false;
    }

    mappings_.insert(std::make_pair(name, spatial_ref));

    RCLCPP_INFO_STREAM(this->get_logger(), "[GeoTF] Added EPSG Code " << id << " as frame " << name);
    return true;
}

// Add a coordinate frame by a Geo Coordinate System
// Prominent example: WGS84
bool GeodeticConverter::addFrameByGCSCode(const std::string &name,
                                          const std::string &gcscode) {
    if (mappings_.count(name)) {
        return false;
    }

    // Create Spatial Reference from well known code
    auto spatial_ref = std::make_shared<OGRSpatialReference>();
    OGRErr err = spatial_ref->SetWellKnownGeogCS(gcscode.c_str());
    if (err != OGRERR_NONE) {
        std::cout << "ERROR" << err << std::endl;
        return false;
    }

    mappings_.insert(std::make_pair(name, spatial_ref));
    RCLCPP_INFO_STREAM(this->get_logger(), "[GeoTF] Added GCS Code " << gcscode << " as frame " << name);
    return true;
}

// Add a frame by UTM Zone
// zone is the UTM zone (e.g. switzerland is in Zone 32)
// north is true for northern hemisphere zones.
bool GeodeticConverter::addFrameByUTM(const std::string &name, const uint zone,
                                      const bool north) {
    // Create Spatial Reference from UTM Zone
    auto spatial_ref = std::make_shared<OGRSpatialReference>();

    spatial_ref->SetWellKnownGeogCS("WGS84");
    spatial_ref->SetUTM(zone, north);

    mappings_.insert(std::make_pair(name, spatial_ref));
    RCLCPP_INFO_STREAM(this->get_logger(), "[GeoTF] Added UTM " << zone << "/" << (north ? "N" : "S")
                                                                << " as frame " << name);
    return true;
}

// Writes a list of all frame definition to console
void GeodeticConverter::writeDebugInfo() const {
    for (auto key : mappings_) {
        std::cout << key.first << std::endl;
        char *pszWKT = NULL;
        key.second->exportToWkt(&pszWKT);
        printf("%s\n", pszWKT);
        CPLFree(pszWKT);

        std::cout << std::endl
                  << std::endl;
    }
}

// Creates a new ENU Frame with its origin at the given
// Location (lat, lon, alt)
// Where (lon,lat,alt) are defined w.r.t. WGS84
bool GeodeticConverter::addFrameByENUOrigin(const std::string &name,
                                            const double lat, const double lon,
                                            const double alt) {
    // Create Spatial Reference from ENU origin
    auto spatial_ref = std::make_shared<OGRSpatialReference>();

    // ENU Frame based on GPS coordinates
    spatial_ref->SetWellKnownGeogCS("WGS84");
    spatial_ref->SetOrthographic(lat, lon, 0.0, 0.0);

    altitude_offsets_.insert(std::make_pair(name, alt));

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[GeoTF] Added ENUOrigin " << lat << "/" << lon << "/" << alt
                                                  << " as frame " << name);
    mappings_.insert(std::make_pair(name, spatial_ref));
    return true;
}

bool GeodeticConverter::addFrameByWKT(const std::string &name,
                                      const std::string &wktformat) {
    // Create Spatial Reference from well known code
    auto spatial_ref = std::make_shared<OGRSpatialReference>();

    std::vector<char> mutable_cstr(wktformat.c_str(),
                                   wktformat.c_str() + wktformat.size() + 1);
    const char *data = mutable_cstr.data();

    OGRErr err = spatial_ref->importFromWkt(&data);
    if (err != OGRERR_NONE) {
        std::cout << "ERROR" << err << std::endl;
        return false;
    }
    mappings_.insert(std::make_pair(name, spatial_ref));

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[GeoTF] Added WKT " << wktformat << " as frame " << name);
    return true;
}

// Checks if two geo frames can be converted
bool GeodeticConverter::canConvert(const std::string &input_frame,
                                   const std::string &output_frame) {
    return checkTransform(input_frame, output_frame);
}

void GeodeticConverter::removeFrame(const std::string &name) {
    size_t num_erased = mappings_.erase(name);
    altitude_offsets_.erase(name);

    for (auto it = mappings_.begin(); it != mappings_.end(); ++it) {
        transforms_.erase(std::make_pair(name, it->first));
        transforms_.erase(std::make_pair(it->first, name));
    }

    if (num_erased > 0) RCLCPP_INFO(this->get_logger(),
                                    "[GeoTF] Erased %s from frame mappings.",
                                    name.c_str());
}

bool GeodeticConverter::hasFrame(const std::string &name) {
    return mappings_.find(name) != mappings_.end();
}

// Converts Pose from one input_frame to output_frame
// Both frames are assumed to be geoframes
// Currently, Attitude is not adjusted.
bool GeodeticConverter::convert(const std::string &input_frame,
                                const Eigen::Affine3d &input,
                                const std::string &output_frame,
                                Eigen::Affine3d *output) {
    *output = input;
    Eigen::Vector3d translation = input.translation();
    bool result =
        convert(input_frame, input.translation(), output_frame, &translation);

    if (!result) {
        return false;
    }
    output->translation() = translation;
    return true;
}

// Converts Position from one input_frame to output_frame
// Both frames are assumed to be geoframes
bool GeodeticConverter::convert(const std::string &input_frame,
                                const Eigen::Vector3d &input,
                                const std::string &output_frame,
                                Eigen::Vector3d *output) {
    OGRCoordinateTransformationPtr transform;

    if (!getTransform(input_frame, output_frame, &transform)) {
        return false;
    }

    // copy data
    *output = input;

    // subtract static offset for input frame if it has one
    if (altitude_offsets_.count(input_frame)) {
        output->z() += altitude_offsets_.at(input_frame);
    }

    // if input system is a geographic coordinate system, switch x and y.
    // We assume that IsGeographic is true for non-ENU systems
    // GDAL default is x = lon, y = lat, but we want it the other way around
    // GDAL default for enu is x=e, y= n, which we do not want to switch
    // This changed in GDAL >=3, so swap is only needed for older versions.
    bool swap_needed_input =
        transform->GetSourceCS()->IsGeographic() && (GDAL_VERSION_MAJOR < 3);

    if (swap_needed_input) {
        std::swap(output->x(), output->y());
    }

    bool transformed = transform->Transform(1, output->data(), output->data() + 1,
                                            output->data() + 2);

    if (!transformed) {
        return false;
    }

    // reverse switch if necessary
    bool swap_needed_output =
        transform->GetTargetCS()->IsGeographic() && (GDAL_VERSION_MAJOR < 3);
    if (swap_needed_output) {
        std::swap(output->x(), output->y());
    }
    // add static offset for output frame if it has one
    if (altitude_offsets_.count(output_frame)) {
        output->z() -= altitude_offsets_.at(output_frame);
    }

    return true;
}

// Converts a Pose in a geoframe to a pose in a tf frame
bool GeodeticConverter::convertToTf(const std::string &geo_input_frame,
                                    const Eigen::Affine3d &input,
                                    const std::string &tf_output_frame,
                                    Eigen::Affine3d *output,
                                    const rclcpp::Time &time) {
    if (!tf_mapping_) {
        RCLCPP_WARN(this->get_logger(), "[GeoTf] No TF mapping defined, canceling convertToTf");
        return false;
    }

    std::string tf_connection_frame = tf_mapping_->second;
    std::string geotf_connection_frame = tf_mapping_->first;
    Eigen::Affine3d tf_connection_value;

    // Convert from whatever geo frame to geotf_connection_frame
    bool result = convert(geo_input_frame, input, geotf_connection_frame,
                          &tf_connection_value);

    if (!result) {
        return false;
    }

    // Convert from tf_connection_frame to tf_output_frame.
    if (!buffer_->canTransform(tf_output_frame, tf_connection_frame, time)) {
        return false;
    }

    geometry_msgs::msg::TransformStamped tf_T_O_C;  // transform connection to output.
    Eigen::Affine3d eigen_T_O_C;
    try {
        tf_T_O_C = buffer_->lookupTransform(tf_output_frame, tf_connection_frame, time);
    } catch (std::exception &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), "[GeoTF] Error in tf connection" << ex.what());
        return false;
    }
    eigen_T_O_C = tf2::transformToEigen(tf_T_O_C);

    *output = eigen_T_O_C * tf_connection_value;
    return true;
}

// Publishes a geolocation as a tf frame
bool GeodeticConverter::publishAsTf(const std::string &geo_input_frame,
                                    const Eigen::Vector3d &input,
                                    const std::string &frame_name) {
    Eigen::Affine3d affine(Eigen::Affine3d::Identity());
    affine.translation() = input;
    return publishAsTf(geo_input_frame, affine, frame_name);
}

// Publishes a geolocation as a tf frame
bool GeodeticConverter::publishAsTf(const std::string &geo_input_frame,
                                    const Eigen::Affine3d &input,
                                    const std::string &frame_name) {
    if (!tf_mapping_) {
        RCLCPP_WARN(this->get_logger(), "[GeoTf] No TF mapping defined, canceling convertAsTf");
        return false;
    }

    std::string tf_connection_frame = tf_mapping_->second;
    std::string geotf_connection_frame = tf_mapping_->first;

    Eigen::Affine3d input_connection;
    bool result = convert(geo_input_frame, input, geotf_connection_frame,
                          &input_connection);

    if (!result) {
        return false;
    }

    geometry_msgs::msg::TransformStamped tf_input;
    tf_input = tf2::eigenToTransform(input_connection);
    tf_input.header.stamp = this->get_clock()->now();
    tf_input.header.frame_id = tf_connection_frame;
    tf_input.child_frame_id = frame_name;
    broadcaster_->sendTransform(tf_input);
    return true;
}

// Convets a Pose in a TF to a pose in a Geo frame
bool GeodeticConverter::convertFromTf(const std::string &tf_input_frame,
                                      const Eigen::Affine3d &input,
                                      const std::string &geo_output_frame,
                                      Eigen::Affine3d *output,
                                      const rclcpp::Time &time) {
    if (!tf_mapping_) {
        RCLCPP_WARN(this->get_logger(), "[GeoTf] No TF mapping defined, canceling convertFromTf");
        return false;
    }

    Eigen::Affine3d tf_connection_value;
    // convert from tf input to
    std::string tf_connection_frame = tf_mapping_->second;
    std::string geotf_connection_frame = tf_mapping_->first;

    // Convert from tf_input_frame  to tf_connection_frame
    if (!buffer_->canTransform(tf_connection_frame, tf_input_frame, time)) {
        return false;
    }

    // add exception handling etc.
    geometry_msgs::msg::TransformStamped tf_T_C_I;  // transform input to connection
    Eigen::Affine3d eigen_T_C_I;

    try {
        tf_T_C_I = buffer_->lookupTransform(tf_connection_frame, tf_input_frame, time);
    } catch (std::exception &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), "[GeoTF] Error in tf connection" << ex.what());
        return false;
    }
    eigen_T_C_I = tf2::transformToEigen(tf_T_C_I);

    tf_connection_value = eigen_T_C_I * input;

    // convert from corresponding
    return convert(geotf_connection_frame, tf_connection_value, geo_output_frame,
                   output);
}

bool GeodeticConverter::getTransform(
    const std::string &input_frame, const std::string &output_frame,
    OGRCoordinateTransformationPtr *transform) {
    if (!checkTransform(input_frame, output_frame)) {
        return false;
    }

    *transform = transforms_.at(std::make_pair(input_frame, output_frame));
    return true;
}

bool GeodeticConverter::checkTransform(const std::string &input_frame,
                                       const std::string &output_frame) {
    TransformId tf_id = std::make_pair(input_frame, output_frame);

    // Check if we already cached transform
    if (transforms_.count(tf_id)) {
        return true;
    }

    // check if we have both frames defined
    if (mappings_.count(tf_id.first) + mappings_.count(tf_id.second) != 2) {
        return false;
    }

    // Create transform
    OGRCoordinateTransformationPtr transform(OGRCreateCoordinateTransformation(
        mappings_.at(tf_id.first).get(), mappings_.at(tf_id.second).get()));

    // if invalid
    if (transform.get() == nullptr) {
        return false;
    }

    // If all goes well
    transforms_.insert(std::make_pair(tf_id, transform));
    return true;
}
}  // namespace geotf
