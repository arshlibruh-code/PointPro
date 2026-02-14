#import "LAZBridge.h"

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "../Vendor/lazperf/cpp/lazperf/writers.hpp"
#include "../Vendor/lazperf/cpp/lazperf/readers.hpp"
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

static void setError(char *error_buf, uint32_t error_buf_len, const char *message)
{
    if (!error_buf || error_buf_len == 0) {
        return;
    }
    if (!message) {
        message = "Unknown error";
    }
    std::snprintf(error_buf, error_buf_len, "%s", message);
}

static int rgbOffsetForFormat(int format)
{
    switch (format)
    {
    case 2: return 20;
    case 3: return 28;
    case 5: return 28;
    case 7: return 30;
    case 8: return 30;
    case 10: return 30;
    default: return -1;
    }
}

static int intensityOffsetForFormat(int format)
{
    switch (format)
    {
    case 0: case 1: case 2: case 3: case 4: case 5:
    case 6: case 7: case 8: case 9: case 10:
        return 12;
    default:
        return -1;
    }
}

static int classificationOffsetForFormat(int format)
{
    switch (format)
    {
    case 0: case 1: case 2: case 3: case 4: case 5:
        return 15;
    case 6: case 7: case 8: case 9: case 10:
        return 16;
    default:
        return -1;
    }
}

static int basePointRecordLengthForFormat(int format)
{
    switch (format)
    {
    case 0: return 20;
    case 1: return 28;
    case 2: return 26;
    case 3: return 34;
    case 4: return 57;
    case 5: return 63;
    case 6: return 30;
    case 7: return 36;
    case 8: return 38;
    case 9: return 59;
    case 10: return 67;
    default: return -1;
    }
}

static uint32_t spatialHashIndex(float x, float y, float z, float voxel_size, uint32_t max_voxels)
{
    if (max_voxels == 0) {
        return 0;
    }
    const float safeVoxel = (voxel_size > 1e-6f) ? voxel_size : 0.005f;
    const int32_t ix = static_cast<int32_t>(std::floor(x / safeVoxel));
    const int32_t iy = static_cast<int32_t>(std::floor(y / safeVoxel));
    const int32_t iz = static_cast<int32_t>(std::floor(z / safeVoxel));
    const uint32_t hx = static_cast<uint32_t>(ix) * 73856093u;
    const uint32_t hy = static_cast<uint32_t>(iy) * 19349663u;
    const uint32_t hz = static_cast<uint32_t>(iz) * 83492791u;
    return (hx ^ hy ^ hz) % max_voxels;
}

#pragma pack(push, 1)
struct PPLVoxelPod
{
    float x;
    float y;
    float z;
    float confidence;
    float r;
    float g;
    float b;
    float sampleCount;
};
#pragma pack(pop)

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

bool pp_laz_create_snapshot(
    const char *path_utf8,
    uint32_t max_points,
    PPProgressCallback progress_cb,
    PPCancelCallback cancel_cb,
    void *callback_context,
    uint8_t **out_bytes,
    uint32_t *out_size,
    uint32_t *out_point_count,
    char *error_buf,
    uint32_t error_buf_len
) {
    if (!out_bytes || !out_size || !out_point_count) {
        setError(error_buf, error_buf_len, "Output pointers are required");
        return false;
    }
    *out_bytes = nullptr;
    *out_size = 0;
    *out_point_count = 0;

    if (!path_utf8 || !path_utf8[0]) {
        setError(error_buf, error_buf_len, "Input file path is empty");
        return false;
    }

    try {
        lazperf::reader::named_file reader{std::string(path_utf8)};
        const lazperf::header14& header = reader.header();

        const uint64_t totalPoints = reader.pointCount();
        if (totalPoints == 0) {
            setError(error_buf, error_buf_len, "Point cloud contains zero points");
            return false;
        }

        const uint32_t targetPoints = max_points > 0 ? max_points : 2'000'000;
        uint64_t stride = 1;
        if (totalPoints > targetPoints) {
            stride = (totalPoints + targetPoints - 1) / targetPoints;
        }

        const int pointFormat = header.pointFormat();
        const int rgbOffset = rgbOffsetForFormat(pointFormat);
        const int intensityOffset = intensityOffsetForFormat(pointFormat);
        const int classificationOffset = classificationOffsetForFormat(pointFormat);
        const size_t recordLength = header.point_record_length;
        if (recordLength < 12) {
            setError(error_buf, error_buf_len, "Unsupported point record length");
            return false;
        }

        std::vector<char> pointBytes(recordLength);
        std::vector<uint8_t> snapshot;
        const uint32_t magic = 0x50435331; // PCS1
        const uint32_t version = 1;
        const size_t headerSize = sizeof(uint32_t) * 3;
        const size_t entrySize = sizeof(uint32_t) + sizeof(PPLVoxelPod);
        const size_t reserveCount = static_cast<size_t>(std::min<uint64_t>(targetPoints, totalPoints));
        snapshot.reserve(headerSize + (reserveCount * entrySize));

        auto appendBytes = [&](const void *ptr, size_t count)
        {
            const uint8_t *b = reinterpret_cast<const uint8_t *>(ptr);
            snapshot.insert(snapshot.end(), b, b + count);
        };

        appendBytes(&magic, sizeof(magic));
        appendBytes(&version, sizeof(version));
        const size_t countOffset = snapshot.size();
        uint32_t countPlaceholder = 0;
        appendBytes(&countPlaceholder, sizeof(countPlaceholder));

        auto emitProgress = [&](float value)
        {
            if (!progress_cb) {
                return;
            }
            if (!std::isfinite(value)) {
                value = 0.0f;
            }
            if (value < 0.0f) value = 0.0f;
            if (value > 1.0f) value = 1.0f;
            progress_cb(value, callback_context);
        };

        emitProgress(0.0f);
        uint32_t outCount = 0;
        bool hasBounds = false;
        float minX = 0.0f;
        float minY = 0.0f;
        float minZ = 0.0f;
        float maxX = 0.0f;
        float maxY = 0.0f;
        float maxZ = 0.0f;
        constexpr uint64_t kProgressStep = 32768;
        for (uint64_t i = 0; i < totalPoints; ++i) {
            if ((i % 4096) == 0 && cancel_cb && cancel_cb(callback_context)) {
                setError(error_buf, error_buf_len, "Decode cancelled.");
                return false;
            }

            reader.readPoint(pointBytes.data());
            if (i % stride != 0) {
                if ((i % kProgressStep) == 0 || (i + 1) == totalPoints) {
                    emitProgress(static_cast<float>(i + 1) / static_cast<float>(totalPoints));
                }
                continue;
            }
            if (outCount >= targetPoints) {
                emitProgress(1.0f);
                break;
            }

            const int32_t xi = lazperf::utils::unpack<int32_t>(pointBytes.data());
            const int32_t yi = lazperf::utils::unpack<int32_t>(pointBytes.data() + 4);
            const int32_t zi = lazperf::utils::unpack<int32_t>(pointBytes.data() + 8);

            float r = 1.0f;
            float g = 1.0f;
            float b = 1.0f;
            float intensityNormalized = 1.0f;
            float classificationNormalized = 0.0f;
            if (rgbOffset >= 0 && (rgbOffset + 6) <= static_cast<int>(recordLength)) {
                const uint16_t rr = lazperf::utils::unpack<uint16_t>(pointBytes.data() + rgbOffset);
                const uint16_t gg = lazperf::utils::unpack<uint16_t>(pointBytes.data() + rgbOffset + 2);
                const uint16_t bb = lazperf::utils::unpack<uint16_t>(pointBytes.data() + rgbOffset + 4);
                r = static_cast<float>(rr) / 65535.0f;
                g = static_cast<float>(gg) / 65535.0f;
                b = static_cast<float>(bb) / 65535.0f;
            }
            if (intensityOffset >= 0 && (intensityOffset + 2) <= static_cast<int>(recordLength)) {
                const uint16_t ii = lazperf::utils::unpack<uint16_t>(pointBytes.data() + intensityOffset);
                intensityNormalized = static_cast<float>(ii) / 65535.0f;
            }
            if (classificationOffset >= 0 && (classificationOffset + 1) <= static_cast<int>(recordLength)) {
                const uint8_t cls = static_cast<uint8_t>(pointBytes[classificationOffset]);
                classificationNormalized = static_cast<float>(cls) / 255.0f;
            }

            const float worldX = static_cast<float>(header.scale.x * static_cast<double>(xi) + header.offset.x);
            const float worldY = static_cast<float>(header.scale.y * static_cast<double>(yi) + header.offset.y);
            const float worldZ = static_cast<float>(header.scale.z * static_cast<double>(zi) + header.offset.z);

            // Align imported geospatial Z-up coordinates to app's Y-up convention.
            PPLVoxelPod voxel;
            voxel.x = worldX;
            voxel.y = worldZ;
            voxel.z = -worldY;
            voxel.confidence = intensityNormalized;
            voxel.r = r;
            voxel.g = g;
            voxel.b = b;
            voxel.sampleCount = 1.0f + (classificationNormalized * 255.0f);

            const uint32_t index = outCount;
            appendBytes(&index, sizeof(index));
            appendBytes(&voxel, sizeof(voxel));
            outCount++;

            if (!hasBounds) {
                minX = maxX = voxel.x;
                minY = maxY = voxel.y;
                minZ = maxZ = voxel.z;
                hasBounds = true;
            } else {
                minX = std::min(minX, voxel.x);
                minY = std::min(minY, voxel.y);
                minZ = std::min(minZ, voxel.z);
                maxX = std::max(maxX, voxel.x);
                maxY = std::max(maxY, voxel.y);
                maxZ = std::max(maxZ, voxel.z);
            }

            if ((i % kProgressStep) == 0 || (i + 1) == totalPoints) {
                emitProgress(static_cast<float>(i + 1) / static_cast<float>(totalPoints));
            }
        }

        if (outCount == 0) {
            setError(error_buf, error_buf_len, "No valid points decoded from file");
            return false;
        }

        std::memcpy(snapshot.data() + countOffset, &outCount, sizeof(outCount));

        if (hasBounds) {
            const float centerX = (minX + maxX) * 0.5f;
            const float centerY = (minY + maxY) * 0.5f;
            const float centerZ = (minZ + maxZ) * 0.5f;

            for (size_t entryOffset = headerSize; entryOffset + entrySize <= snapshot.size(); entryOffset += entrySize) {
                const size_t voxelOffset = entryOffset + sizeof(uint32_t);
                PPLVoxelPod pod;
                std::memcpy(&pod, snapshot.data() + voxelOffset, sizeof(PPLVoxelPod));
                pod.x -= centerX;
                pod.y -= centerY;
                pod.z -= centerZ;
                std::memcpy(snapshot.data() + voxelOffset, &pod, sizeof(PPLVoxelPod));
            }
        }

        uint8_t *buffer = reinterpret_cast<uint8_t *>(std::malloc(snapshot.size()));
        if (!buffer) {
            setError(error_buf, error_buf_len, "Out of memory while creating snapshot");
            return false;
        }
        std::memcpy(buffer, snapshot.data(), snapshot.size());
        *out_bytes = buffer;
        *out_size = static_cast<uint32_t>(snapshot.size());
        *out_point_count = outCount;
        emitProgress(1.0f);
        return true;
    } catch (const std::exception& e) {
        setError(error_buf, error_buf_len, e.what());
        return false;
    } catch (...) {
        setError(error_buf, error_buf_len, "Unknown LAZ decode failure");
        return false;
    }
}

bool pp_laz_decompress_chunk_to_snapshot(
    const uint8_t *chunk_bytes,
    uint32_t chunk_size,
    uint8_t point_format,
    uint16_t point_record_length,
    uint32_t point_count,
    double scale_x,
    double scale_y,
    double scale_z,
    double offset_x,
    double offset_y,
    double offset_z,
    float center_x,
    float center_y,
    float center_z,
    float voxel_size,
    uint32_t max_voxels,
    uint8_t **out_bytes,
    uint32_t *out_size,
    uint32_t *out_point_count,
    char *error_buf,
    uint32_t error_buf_len
)
{
    if (!chunk_bytes || chunk_size == 0) {
        setError(error_buf, error_buf_len, "Chunk bytes are empty");
        return false;
    }
    if (!out_bytes || !out_size || !out_point_count) {
        setError(error_buf, error_buf_len, "Output pointers are required");
        return false;
    }
    *out_bytes = nullptr;
    *out_size = 0;
    *out_point_count = 0;

    const int format = static_cast<int>(point_format);
    const int baseLength = basePointRecordLengthForFormat(format);
    if (baseLength < 0) {
        setError(error_buf, error_buf_len, "Unsupported point format");
        return false;
    }
    if (point_record_length < 12 || point_record_length < static_cast<uint16_t>(baseLength)) {
        setError(error_buf, error_buf_len, "Invalid point record length");
        return false;
    }
    const int ebCount = static_cast<int>(point_record_length) - baseLength;
    const int rgbOffset = rgbOffsetForFormat(format);
    const int intensityOffset = intensityOffsetForFormat(format);
    const int classificationOffset = classificationOffsetForFormat(format);

    const uint32_t safeMaxVoxels = max_voxels > 0 ? max_voxels : 2'000'000;
    const uint32_t reservePoints = point_count > 0 ? point_count : 1;

    try {
        std::vector<uint8_t> snapshot;
        const uint32_t magic = 0x50435331; // PCS1
        const uint32_t version = 1;
        const size_t headerSize = sizeof(uint32_t) * 3;
        const size_t entrySize = sizeof(uint32_t) + sizeof(PPLVoxelPod);
        snapshot.reserve(headerSize + (static_cast<size_t>(reservePoints) * entrySize));

        auto appendBytes = [&](const void *ptr, size_t count)
        {
            const uint8_t *b = reinterpret_cast<const uint8_t *>(ptr);
            snapshot.insert(snapshot.end(), b, b + count);
        };

        appendBytes(&magic, sizeof(magic));
        appendBytes(&version, sizeof(version));
        const size_t countOffset = snapshot.size();
        uint32_t countPlaceholder = 0;
        appendBytes(&countPlaceholder, sizeof(countPlaceholder));

        lazperf::reader::chunk_decompressor decompressor(format, ebCount, reinterpret_cast<const char *>(chunk_bytes));
        std::vector<char> pointBytes(point_record_length);

        uint32_t written = 0;
        for (uint32_t i = 0; i < point_count; ++i) {
            decompressor.decompress(pointBytes.data());

            const int32_t xi = lazperf::utils::unpack<int32_t>(pointBytes.data());
            const int32_t yi = lazperf::utils::unpack<int32_t>(pointBytes.data() + 4);
            const int32_t zi = lazperf::utils::unpack<int32_t>(pointBytes.data() + 8);

            const float worldX = static_cast<float>(scale_x * static_cast<double>(xi) + offset_x);
            const float worldY = static_cast<float>(scale_y * static_cast<double>(yi) + offset_y);
            const float worldZ = static_cast<float>(scale_z * static_cast<double>(zi) + offset_z);

            // Keep center subtraction in the same (Z-up) frame, then remap to app Y-up.
            float x = worldX - center_x;
            float y = worldZ - center_z;
            float z = -(worldY - center_y);

            float r = 1.0f;
            float g = 1.0f;
            float b = 1.0f;
            float intensityNormalized = 1.0f;
            float classificationNormalized = 0.0f;
            if (rgbOffset >= 0 && (rgbOffset + 6) <= static_cast<int>(point_record_length)) {
                const uint16_t rr = lazperf::utils::unpack<uint16_t>(pointBytes.data() + rgbOffset);
                const uint16_t gg = lazperf::utils::unpack<uint16_t>(pointBytes.data() + rgbOffset + 2);
                const uint16_t bb = lazperf::utils::unpack<uint16_t>(pointBytes.data() + rgbOffset + 4);
                r = static_cast<float>(rr) / 65535.0f;
                g = static_cast<float>(gg) / 65535.0f;
                b = static_cast<float>(bb) / 65535.0f;
            }
            if (intensityOffset >= 0 && (intensityOffset + 2) <= static_cast<int>(point_record_length)) {
                const uint16_t ii = lazperf::utils::unpack<uint16_t>(pointBytes.data() + intensityOffset);
                intensityNormalized = static_cast<float>(ii) / 65535.0f;
            }
            if (classificationOffset >= 0 && (classificationOffset + 1) <= static_cast<int>(point_record_length)) {
                const uint8_t cls = static_cast<uint8_t>(pointBytes[classificationOffset]);
                classificationNormalized = static_cast<float>(cls) / 255.0f;
            }

            PPLVoxelPod voxel;
            voxel.x = x;
            voxel.y = y;
            voxel.z = z;
            voxel.confidence = intensityNormalized;
            voxel.r = r;
            voxel.g = g;
            voxel.b = b;
            voxel.sampleCount = 1.0f + (classificationNormalized * 255.0f);

            const uint32_t index = spatialHashIndex(x, y, z, voxel_size, safeMaxVoxels);
            appendBytes(&index, sizeof(index));
            appendBytes(&voxel, sizeof(voxel));
            written++;
        }

        if (written == 0) {
            setError(error_buf, error_buf_len, "Decoded zero points from chunk");
            return false;
        }

        std::memcpy(snapshot.data() + countOffset, &written, sizeof(written));
        uint8_t *buffer = reinterpret_cast<uint8_t *>(std::malloc(snapshot.size()));
        if (!buffer) {
            setError(error_buf, error_buf_len, "Out of memory while building chunk snapshot");
            return false;
        }
        std::memcpy(buffer, snapshot.data(), snapshot.size());
        *out_bytes = buffer;
        *out_size = static_cast<uint32_t>(snapshot.size());
        *out_point_count = written;
        return true;
    } catch (const std::exception& e) {
        setError(error_buf, error_buf_len, e.what());
        return false;
    } catch (...) {
        setError(error_buf, error_buf_len, "Unknown chunk decode failure");
        return false;
    }
}

void pp_free_buffer(void *ptr)
{
    if (ptr) {
        std::free(ptr);
    }
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
