#ifndef LAZBridge_h
#define LAZBridge_h

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PPLOpaqueLAZWriter PPLOpaqueLAZWriter;
typedef void (*PPProgressCallback)(float progress, void *context);
typedef bool (*PPCancelCallback)(void *context);

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
);

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
);

bool pp_laz_writer_close(PPLOpaqueLAZWriter *writer);
const char *pp_laz_writer_last_error(PPLOpaqueLAZWriter *writer);
void pp_laz_writer_destroy(PPLOpaqueLAZWriter *writer);

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
);

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
);

void pp_free_buffer(void *ptr);

#ifdef __cplusplus
}
#endif

#endif /* LAZBridge_h */
