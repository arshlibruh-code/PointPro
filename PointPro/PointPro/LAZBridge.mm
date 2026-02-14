#import "LAZBridge.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include "../Vendor/lazperf/cpp/lazperf/writers.hpp"
#include "../Vendor/lazperf/cpp/lazperf/las.hpp"

struct PPLOpaqueLAZWriter {
    std::unique_ptr<lazperf::writer::named_file> file;
    double scaleX {0.001};
    double scaleY {0.001};
    double scaleZ {0.001};
    double offsetX {0.0};
    double offsetY {0.0};
    double offsetZ {0.0};
    bool closed {false};
    std::string lastError;
};

static int32_t quantizeCoordinate(double value, double scale, double offset) {
    const double normalized = (value - offset) / scale;
    if (!std::isfinite(normalized)) {
        return 0;
    }

    const double rounded = std::llround(normalized);
    if (rounded > static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return std::numeric_limits<int32_t>::max();
    }
    if (rounded < static_cast<double>(std::numeric_limits<int32_t>::min())) {
        return std::numeric_limits<int32_t>::min();
    }
    return static_cast<int32_t>(rounded);
}

PPLOpaqueLAZWriter *pp_laz_writer_create(
    const char *path_utf8,
    double scale_x,
    double scale_y,
    double scale_z,
    double offset_x,
    double offset_y,
    double offset_z,
    uint32_t chunk_size,
    const char *wkt_utf8
) {
    if (!path_utf8 || !path_utf8[0]) {
        return nullptr;
    }

    auto writer = std::make_unique<PPLOpaqueLAZWriter>();
    writer->scaleX = (scale_x > 0.0) ? scale_x : 0.001;
    writer->scaleY = (scale_y > 0.0) ? scale_y : 0.001;
    writer->scaleZ = (scale_z > 0.0) ? scale_z : 0.001;
    writer->offsetX = offset_x;
    writer->offsetY = offset_y;
    writer->offsetZ = offset_z;

    try {
        lazperf::writer::named_file::config config(
            lazperf::vector3(writer->scaleX, writer->scaleY, writer->scaleZ),
            lazperf::vector3(writer->offsetX, writer->offsetY, writer->offsetZ),
            chunk_size > 0 ? chunk_size : lazperf::DefaultChunkSize
        );
        config.pdrf = 2; // point10 + RGB
        config.minor_version = 4;
        config.extra_bytes = 0;
        config.wkt = wkt_utf8 ? std::string(wkt_utf8) : std::string();

        writer->file = std::make_unique<lazperf::writer::named_file>(std::string(path_utf8), config);
    } catch (const std::exception& e) {
        writer->lastError = e.what();
        return nullptr;
    } catch (...) {
        writer->lastError = "Unknown LAZ writer initialization failure";
        return nullptr;
    }

    return writer.release();
}

bool pp_laz_writer_write_point(
    PPLOpaqueLAZWriter *writer,
    double x,
    double y,
    double z,
    uint16_t red,
    uint16_t green,
    uint16_t blue,
    uint16_t intensity,
    uint8_t classification
) {
    if (!writer || !writer->file || writer->closed) {
        return false;
    }

    try {
        lazperf::las::point10 point;
        point.x = quantizeCoordinate(x, writer->scaleX, writer->offsetX);
        point.y = quantizeCoordinate(y, writer->scaleY, writer->offsetY);
        point.z = quantizeCoordinate(z, writer->scaleZ, writer->offsetZ);
        point.intensity = intensity;
        point.return_number = 1;
        point.number_of_returns_of_given_pulse = 1;
        point.scan_direction_flag = 0;
        point.edge_of_flight_line = 0;
        point.classification = classification;
        point.scan_angle_rank = 0;
        point.user_data = 0;
        point.point_source_ID = 0;

        lazperf::las::rgb color(red, green, blue);

        char packed[26];
        std::memset(packed, 0, sizeof(packed));
        point.pack(packed);
        color.pack(packed + 20);

        writer->file->writePoint(packed);
        return true;
    } catch (const std::exception& e) {
        writer->lastError = e.what();
        return false;
    } catch (...) {
        writer->lastError = "Unknown LAZ write failure";
        return false;
    }
}

bool pp_laz_writer_close(PPLOpaqueLAZWriter *writer) {
    if (!writer) {
        return false;
    }
    if (writer->closed) {
        return true;
    }

    if (!writer->file) {
        return false;
    }

    try {
        writer->file->close();
        writer->closed = true;
        return true;
    } catch (const std::exception& e) {
        writer->lastError = e.what();
        return false;
    } catch (...) {
        writer->lastError = "Unknown LAZ close failure";
        return false;
    }
}

const char *pp_laz_writer_last_error(PPLOpaqueLAZWriter *writer) {
    if (!writer || writer->lastError.empty()) {
        return "";
    }
    return writer->lastError.c_str();
}

void pp_laz_writer_destroy(PPLOpaqueLAZWriter *writer) {
    if (!writer) {
        return;
    }

    if (writer->file && !writer->closed) {
        try {
            writer->file->close();
        } catch (...) {
            // Best effort cleanup.
        }
    }

    delete writer;
}

// lazperf implementation units (single TU integration)
#include "../Vendor/lazperf/cpp/lazperf/charbuf.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_byte10.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_byte14.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_gpstime10.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_nir14.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_point10.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_point14.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_rgb10.cpp"
#include "../Vendor/lazperf/cpp/lazperf/detail/field_rgb14.cpp"
#include "../Vendor/lazperf/cpp/lazperf/filestream.cpp"
#include "../Vendor/lazperf/cpp/lazperf/header.cpp"
#include "../Vendor/lazperf/cpp/lazperf/lazperf.cpp"
#include "../Vendor/lazperf/cpp/lazperf/readers.cpp"
#include "../Vendor/lazperf/cpp/lazperf/vlr.cpp"
#include "../Vendor/lazperf/cpp/lazperf/writers.cpp"
